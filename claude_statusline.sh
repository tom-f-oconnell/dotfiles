#!/usr/bin/env bash
# Claude Code's status line (statusLine in claude_settings.json): bars for how much of the
# plan's 5-hour and weekly usage limits is used, and when each resets, then the vim mode
# (in place of the built-in `-- INSERT --` line, which hideVimModeIndicator hides). E.g.:
#   5h ▓▓▓░░░░░░░ 31% (resets 14:05) · 7d ▓▓░░░░░░░░ 18% (resets Thu) · -- INSERT --
#
# Claude Code passes the session's state as JSON on stdin. The usage is only there when
# logged in with a Pro / Max / Ultra plan, and only after the session's first reply.

# (\x1f-separated, as read's IFS would merge runs of a whitespace separator like \t, losing
# empty fields)
IFS=$'\x1f' read -r mode five five_reset week week_reset < <(jq -r '[
    .vim.mode,
    .rate_limits.five_hour.used_percentage, .rate_limits.five_hour.resets_at,
    .rate_limits.seven_day.used_percentage, .rate_limits.seven_day.resets_at
] | map(. // "" | tostring) | join("\u001f")')

printf -v now '%(%s)T' -1

# Prints "<label> <bar> <percent>% (resets <when>)" for one usage window, or nothing if
# Claude Code didn't give it. The bar is green, then yellow from 70%, and red from 90%.
#
# Args:
#   $1: the label.
#   $2: the percent of the limit used.
#   $3: when the window resets, in seconds since the epoch.
function window() {
    local label=$1 percent=$2 reset=$3
    [ -n "$percent" ] || return 0
    printf -v percent '%.0f' "$percent"
    local filled=$(( (percent + 5) / 10 ))
    (( filled > 10 )) && filled=10
    local color='38;5;114'
    if (( percent >= 90 )); then
        color=31
    elif (( percent >= 70 )); then
        color=33
    fi
    local bar= i
    for (( i = 0; i < 10; i++ )); do
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
        out+=" (resets $when)"
    fi
    printf '%s' "$out"
}

parts=()
for p in "$(window 5h "$five" "$five_reset")" "$(window 7d "$week" "$week_reset")"; do
    [ -n "$p" ] && parts+=("$p")
done
# (vim shows nothing for normal mode)
if [ -n "$mode" ] && [ "$mode" != NORMAL ]; then
    parts+=("-- $mode --")
fi

out=
for p in "${parts[@]}"; do
    out+="${out:+ · }$p"
done
printf '%s\n' "$out"
