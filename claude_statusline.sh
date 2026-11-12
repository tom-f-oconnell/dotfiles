#!/usr/bin/env bash
# Claude Code's status line (statusLine in claude_settings.json): the model and its effort
# level, then bars for how much of the plan's 5-hour and weekly usage limits is used, and
# when each resets, then any per-model weekly limits (e.g. Fable's), then the vim mode (in
# place of the built-in `-- INSERT --` line, which hideVimModeIndicator hides). E.g.:
#   Opus 5.5 high · 5h ▓▓▓░░░░░░░ 31% (14:05) · 7d ▓▓▓▓▓▓▓░░░ 73% (Sun) · Fable ▓▓▓▓░░░░░░ 43%
# The bars are narrowed (to no less than MIN_CELLS) as needed to fit the terminal.
#
# Claude Code passes the session's state as JSON on stdin. The 5-hour and weekly usage is
# only there when logged in with a Pro / Max / Ultra plan, and only after the session's
# first reply. The per-model limits aren't in it at all, so they come from the usage
# endpoint /usage reads, fetched in the background (at most every FETCH_EVERY seconds)
# into a cache, so the status line never waits on the network.

FETCH_EVERY=120
# (cached usage older than this isn't shown, e.g. if the login expired)
SHOW_FOR=600
# (how many cells wide each bar is, at most, and at least when narrowed to fit)
MAX_CELLS=10
MIN_CELLS=4
# (columns of the terminal the status line can't use, as Claude Code pads it on each side)
MARGIN=4

shopt -s extglob

cache=${XDG_CACHE_HOME:-$HOME/.cache}/claude_statusline
usage=$cache/usage.json

# (\x1f-separated, as read's IFS would merge runs of a whitespace separator like \t, losing
# empty fields)
IFS=$'\x1f' read -r model effort mode five five_reset week week_reset < <(jq -r '[
    .model.display_name, .effort.level, .vim.mode,
    .rate_limits.five_hour.used_percentage, .rate_limits.five_hour.resets_at,
    .rate_limits.seven_day.used_percentage, .rate_limits.seven_day.resets_at
] | map(. // "" | tostring) | join("\u001f")')

printf -v now '%(%s)T' -1

# Saves the plan's usage, from the endpoint Claude Code's /usage reads, to $usage (with
# the time it was fetched added, as fetched_at). Does nothing without a current claude.ai
# login.
function fetch_usage() {
    local token tmp
    token=$(jq -r --argjson now "$now" '.claudeAiOauth
        | select(.expiresAt / 1000 > $now) | .accessToken // empty' \
        "${CLAUDE_CONFIG_DIR:-$HOME/.claude}/.credentials.json" 2>/dev/null)
    [ -n "$token" ] || return 0
    tmp=$(mktemp "$usage.XXXXXX") || return 0
    # (the token is given on stdin, so it isn't in curl's command line, which any user
    # can see)
    if printf 'Authorization: Bearer %s\nanthropic-beta: oauth-2025-04-20\n' "$token" \
            | curl -sf -m 10 -H @- https://api.anthropic.com/api/oauth/usage \
            | jq -e --argjson now "$now" 'select(.limits) | . + {fetched_at: $now}' \
                > "$tmp"; then
        mv -f "$tmp" "$usage"
    else
        rm -f "$tmp"
    fi
}

# (the time of the last fetch attempt, so failed fetches aren't retried any sooner)
last_fetch=0
read -r last_fetch 2>/dev/null < "$cache/fetched"
if (( now - last_fetch >= FETCH_EVERY )) && mkdir -p "$cache" \
        && printf '%s\n' "$now" > "$cache/fetched"; then
    # (with no output left open, so Claude Code doesn't wait for it)
    fetch_usage < /dev/null > /dev/null 2>&1 &
fi

# Prints "<label> <bar> <percent>% (<when it resets>)" for one usage window. The bar is
# green, then yellow from 70%, and red from 90%.
#
# Args:
#   $1: the label.
#   $2: the percent of the limit used.
#   $3: when the window resets, in seconds since the epoch. If empty, the reset is left
#       out.
#   $4: how many cells wide the bar is.
function window() {
    local label=$1 percent=$2 reset=$3 cells=$4
    printf -v percent '%.0f' "$percent"
    local filled=$(( (percent * cells + 50) / 100 ))
    (( filled > cells )) && filled=$cells
    local color='38;5;114'
    if (( percent >= 90 )); then
        color=31
    elif (( percent >= 70 )); then
        color=33
    fi
    local bar= i
    for (( i = 0; i < cells; i++ )); do
        if (( i < filled )); then bar+='▓'; else bar+='░'; fi
    done
    local out="$label "$'\e['"${color}m${bar}"$'\e[m'" $percent%"
    if [ -n "$reset" ]; then
        # (the time if within a day, else the day)
        local when
        if (( reset - now < 86400 )); then
            printf -v when '%(%H:%M)T' "$reset"
        else
            printf -v when '%(%a)T' "$reset"
        fi
        out+=" ($when)"
    fi
    printf '%s' "$out"
}

# The usage windows to show, as parallel arrays of window()'s first 3 args.
labels=() percents=() resets=()

# Adds a usage window to show, if Claude Code gave its percent used. Takes window()'s first
# 3 args.
function add_window() {
    [ -n "$2" ] || return 0
    labels+=("$1") percents+=("$2") resets+=("$3")
}

add_window 5h "$five" "$five_reset"
add_window 7d "$week" "$week_reset"

# The per-model weekly limits, as "<name>\x1f<percent>\x1f<reset>" lines, from usage
# fetched within SHOW_FOR, and leaving out any whose window has since reset.
if [ -f "$usage" ]; then
    while IFS=$'\x1f' read -r name percent reset; do
        # (these reset with the weekly limit, give or take a second, so only say when
        # they reset if not then)
        if [ -n "$reset" ] && [ -n "$week_reset" ] \
                && (( reset - week_reset < 300 && week_reset - reset < 300 )); then
            reset=
        fi
        add_window "$name" "$percent" "$reset"
    done < <(jq -r --argjson now "$now" --argjson show_for "$SHOW_FOR" '
        select(.fetched_at > $now - $show_for) | .limits[]?
        | select(.kind == "weekly_scoped" and .scope.model.display_name != null
            and .percent != null)
        | [.scope.model.display_name, .percent, (.resets_at // ""
            | sub("\\.[0-9]+"; "") | sub("\\+00:00$"; "Z") | try fromdateiso8601 catch "")]
        | select(.[2] == "" or .[2] > $now)
        | map(tostring) | join("\u001f")' "$usage" 2>/dev/null)
fi

# (vim shows nothing for normal mode)
vim=
if [ -n "$mode" ] && [ "$mode" != NORMAL ]; then
    vim="-- $mode --"
fi

# Prints the status line.
#
# Args:
#   $1: how many cells wide each usage window's bar is.
function line() {
    local out="$model${effort:+ $effort}" i
    for i in "${!labels[@]}"; do
        out+="${out:+ · }$(window "${labels[i]}" "${percents[i]}" "${resets[i]}" "$1")"
    done
    [ -n "$vim" ] && out+="${out:+ · }$vim"
    printf '%s' "$out"
}

out=$(line "$MAX_CELLS")
# (Claude Code gives the terminal's width as COLUMNS)
if [ -n "$COLUMNS" ] && (( ${#labels[@]} )); then
    plain=${out//$'\e['*([0-9;])m}
    over=$(( ${#plain} - (COLUMNS - MARGIN) ))
    if (( over > 0 )); then
        # (taking the same number of cells from each bar, rounding up)
        cells=$(( MAX_CELLS - (over + ${#labels[@]} - 1) / ${#labels[@]} ))
        (( cells < MIN_CELLS )) && cells=$MIN_CELLS
        out=$(line "$cells")
    fi
fi
printf '%s\n' "$out"
