#!/usr/bin/env bash
# Claude Code's status line (statusLine in claude_settings.json): the model (in red if
# Fable) and its effort level, as a cyan bar per level, filled up to it, then bars for how
# much of the plan's 5-hour and weekly usage limits is used, and when each resets, then any
# per-model weekly limits (e.g. Fable's), then as much of the session's name as fits, and
# at the right edge, the CPU, memory, and (if any) GPU use of Claude Code and everything it
# started. E.g.:
#   Opus5.5 ▊▊▊▊▊ 7d ▓▓▓▓░ 73% (Sun) Fable ▓▓░░░ 43% · Fix the pa…  cpu 340% mem 3.4G
# The bars are narrowed (to no less than MIN_CELLS) as needed to fit the rest but the name.
# The vim mode isn't shown (and hideVimModeIndicator hides the built-in `-- INSERT --`).
#
# Claude Code passes the session's state as JSON on stdin. The 5-hour and weekly usage is
# only there when logged in with a Pro / Max / Ultra plan, and only after the session's
# first reply. The per-model limits aren't in it at all, so they come from the usage
# endpoint /usage reads, fetched in the background (at most every FETCH_EVERY seconds)
# into a cache, so the status line never waits on the network.
#
# The CPU, memory and GPU use is likewise sampled in the background, by
# claude_procstats.py (see there for what's counted), at most every SAMPLE_EVERY seconds,
# so it's from the sample before, with CPU use averaged since the one before that. CPU and
# memory turn yellow from 70% of the machine's, and red from 90%. statusLine's
# refreshInterval re-runs this while Claude Code is otherwise idle, e.g. waiting on a job.

FETCH_EVERY=120
# (cached usage older than this isn't shown, e.g. if the login expired)
SHOW_FOR=600
SAMPLE_EVERY=4
# (older samples aren't shown, e.g. if sampling keeps failing)
SHOW_SAMPLE_FOR=30
# (how many cells wide each bar is, at most, and at least when narrowed to fit)
MAX_CELLS=5
MIN_CELLS=2
# (columns of the terminal the status line can't use, as Claude Code pads it on each side)
MARGIN=4
# (the fewest columns to show a cut-off session name in, else it's left out)
MIN_NAME=6

shopt -s extglob

cache=${XDG_CACHE_HOME:-$HOME/.cache}/claude_statusline
usage=$cache/usage.json

# (\x1f-separated, as read's IFS would merge runs of a whitespace separator like \t, losing
# empty fields)
IFS=$'\x1f' read -r model effort five five_reset week week_reset session < <(jq -r '[
    .model.display_name, .effort.level,
    .rate_limits.five_hour.used_percentage, .rate_limits.five_hour.resets_at,
    .rate_limits.seven_day.used_percentage, .rate_limits.seven_day.resets_at,
    (.session_name // "" | gsub("[[:cntrl:]]"; ""))
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

# Sets the variable named $2 to the color (an SGR parameter, or $3 if none) to show
# something at $1 percent (of a limit, or of the machine) in: yellow from 70%, and red from
# 90%. (Variables are set, rather than output printed, by this and the functions for the
# CPU etc. use, as a subshell to capture output from takes a ms or two each.)
function level_color() {
    if (( $1 >= 90 )); then
        printf -v "$2" 31
    elif (( $1 >= 70 )); then
        printf -v "$2" 33
    else
        printf -v "$2" '%s' "$3"
    fi
}

# Prints "<label> <bar> <percent>% (<when it resets>)" for one usage window. The bar is
# green, then as level_color() says.
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
    local color
    level_color "$percent" color '38;5;114'
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

# The model (without spaces, e.g. Opus5.5) and its effort level, as line() starts with
model=${model// /}
head=$model
if [[ ${model,,} == *fable* ]]; then
    head=$'\e[31m'"$model"$'\e[m'
fi
if [ -n "$effort" ]; then
    # (a narrow full-height block per level, filled ones in the cyan a usage bar's ▓ would
    # look like, and the rest as its ░ would, as the terminal draws shades by blending)
    levels=(low medium high xhigh max)
    for i in "${!levels[@]}"; do
        if [ "${levels[i]}" = "$effort" ]; then
            head+=" "$'\e[38;5;73m'
            for (( j = 0; j < ${#levels[@]}; j++ )); do
                (( j == i + 1 )) && head+=$'\e[38;5;23m'
                head+='▊'
            done
            head+=$'\e[m'
            break
        fi
    done
fi

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
# (how many of the windows are the plan's, which are separated by " · ", where the rest are
# by a space)
plan_windows=${#labels[@]}

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

# Claude Code's process: the nearest ancestor run as claude (which CLAUDE_PID also is, if
# set, in the commands it runs). Its name (comm) isn't always claude, e.g. 2.1.325 for a
# background session's, which titles itself "claude bg-spare ..." (as its whole command
# line), and is run by a daemon, also run as claude, but whose usage would include every
# background session's.
claude_pid= p=$PPID
while (( p > 1 )) && [ -e "/proc/$p" ]; do
    # (up to the first NUL, which ends each arg)
    IFS= read -r -d '' arg0 2>/dev/null < "/proc/$p/cmdline"
    arg0=${arg0%% *}
    if [ "${arg0##*/}" = claude ]; then
        claude_pid=$p
        break
    fi
    # (the parent is the 2nd field after the name, which is in parentheses)
    read -r stat < "/proc/$p/stat" || break
    read -r _ p _ <<< "${stat##*') '}"
done
if [ -z "$claude_pid" ] && [ -n "$CLAUDE_PID" ] && [ -d "/proc/$CLAUDE_PID" ]; then
    claude_pid=$CLAUDE_PID
fi

# Sets $size to an amount of memory, given in KiB, in M under 1000M, else in G (to 1
# decimal under 10G), e.g. 412M, 3.4G, 12G.
function size() {
    local kib=$1
    if (( kib < 1000 * 1024 - 512 )); then
        printf -v size '%dM' $(( (kib + 512) / 1024 ))
    elif (( kib < 10 * 1024 * 1024 - 52429 )); then
        local tenths=$(( (kib * 10 + 524288) / 1048576 ))
        printf -v size '%d.%dG' $(( tenths / 10 )) $(( tenths % 10 ))
    else
        printf -v size '%dG' $(( (kib + 524288) / 1048576 ))
    fi
}

# (the color of the CPU etc. use, to set it apart from the session name before it)
PROCS_COLOR='38;5;110'

# Appends to $procs "<label> <value>", in PROCS_COLOR, but with the value in
# level_color()'s color for $3 percent, if any.
function add_level() {
    local color
    level_color "$3" color "$PROCS_COLOR"
    procs+="${procs:+ }"$'\e['"${PROCS_COLOR}m$1 "$'\e['"${color}m$2"$'\e[m'
}

# The CPU, memory and GPU use, e.g. "cpu 340% mem 3.4G gpu 85% 1.2G", from the last sample
procs=
if [ -n "$claude_pid" ]; then
    run=${XDG_RUNTIME_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}}/claude_statusline
    sample=$run/procs-$claude_pid
    sampled=0
    read -r sampled cpu cpus mem mem_total gpu gpu_mem 2>/dev/null < "$sample"
    if (( now - sampled >= SAMPLE_EVERY )) && mkdir -p "$run"; then
        # (in its own session, as Claude Code kills this if it's still running when it next
        # updates the status line)
        setsid -f python3 -I -S "${BASH_SOURCE[0]%/*}/claude_procstats.py" "$claude_pid" \
            "$sample" < /dev/null > /dev/null 2>&1
    fi
    if (( now - sampled < SHOW_SAMPLE_FOR )); then
        # (CPU use is unknown until the second sample)
        [ "$cpu" != - ] && add_level cpu "$cpu%" $(( cpus ? cpu / cpus : 0 ))
        size "$mem"
        add_level mem "$size" $(( mem_total ? mem * 100 / mem_total : 0 ))
        if [ "$gpu" != - ]; then
            size $(( gpu_mem * 1024 ))
            procs+=" "$'\e['"${PROCS_COLOR}mgpu $gpu% $size"$'\e[m'
        fi
    fi
fi

# Prints the model, its effort level, and the usage windows.
#
# Args:
#   $1: how many cells wide each usage window's bar is.
function line() {
    local out=$head i sep
    for i in "${!labels[@]}"; do
        sep=' '
        (( i > 0 && i < plan_windows )) && sep=' · '
        out+="${out:+$sep}$(window "${labels[i]}" "${percents[i]}" "${resets[i]}" "$1")"
    done
    printf '%s' "$out"
}

# Prints how many columns $1 takes up on the terminal.
function width() {
    local plain=${1//$'\e['*([0-9;])m}
    printf '%s' "${#plain}"
}

out=$(line "$MAX_CELLS")
# (Claude Code gives the terminal's width as COLUMNS)
if [ -z "$COLUMNS" ]; then
    [ -n "$session" ] && out+="${out:+ · }$session"
    [ -n "$procs" ] && out+="${out:+ · }$procs"
    printf '%s\n' "$out"
    exit
fi

room=$(( COLUMNS - MARGIN ))
# (the CPU etc. use, with at least a space before it)
right=0
[ -n "$procs" ] && right=$(( $(width "$procs") + 1 ))
over=$(( $(width "$out") + right - room ))
if (( over > 0 && ${#labels[@]} )); then
    # (taking the same number of cells from each bar, rounding up)
    cells=$(( MAX_CELLS - (over + ${#labels[@]} - 1) / ${#labels[@]} ))
    (( cells < MIN_CELLS )) && cells=$MIN_CELLS
    out=$(line "$cells")
fi

if [ -n "$session" ]; then
    left=$(( room - $(width "$out") - 3 - right ))
    if (( ${#session} > left && left >= MIN_NAME )); then
        session="${session:0:left-1}…"
    fi
    (( ${#session} <= left )) && out+=" · $session"
fi

if [ -n "$procs" ]; then
    pad=$(( room - $(width "$out") - right + 1 ))
    (( pad < 1 )) && pad=1
    printf -v spaces '%*s' "$pad" ''
    out+="$spaces$procs"
fi
printf '%s\n' "$out"
