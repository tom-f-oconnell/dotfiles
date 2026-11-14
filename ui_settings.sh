#!/usr/bin/env bash

# Settings for the GNOME desktop (as in Ubuntu 26.04). The Unity / Compiz settings this
# used to make (workspace grid, launcher favorites, sticky edges, indicator-datetime) do
# not exist there, and were removed.

# https://stackoverflow.com/questions/38859145
if grep -q Microsoft /proc/version; then
    # Doesn't make sense to try to change these in WSL.
    exit 0
fi

# See: https://askubuntu.com/questions/558446
# for possible reasons Anaconda / other things could break gsettings
# (so would need to check whether it's broken and work around, since
# I often run things from shells with an anaconda environment loaded)
# (`conda deactivate` seems to get rid of the warning at least...
# could just run that unconditionally, ignoring error in case when conda
# not installed)
# Related to above: I noticed only gsettings w/ conda activated seems
# to have the useful 'describe' command (so it must be a newer version?)

# turn off screen / lock after 30 minutes
gsettings set org.gnome.desktop.session idle-delay 1800

# TODO uncomment once all of these are set up (rhythmbox, and which terminal?). These are
# the GNOME names for what the Unity launcher list had.
#gsettings set org.gnome.shell favorite-apps "['org.gnome.Nautilus.desktop', 'firefox_firefox.desktop', 'org.gnome.Rhythmbox3.desktop', 'libreoffice-calc.desktop', 'libreoffice-writer.desktop', 'org.gnome.Ptyxis.desktop']"

# TODO re-evaluate whether I still want the three settings below (volume keys, key
# repeat, ctrl+q hotkey). Commented 2026-11-13: none were applied on atlas at the time.
# See also migrating_movein_sh.md.

# keybindings, to have consistent keys for volume control across all computers
# (these keys take a list now, so several bindings per function are possible)
# TODO maybe i could detect whether there are native keyboard controls on the
# particular computer, and only set these if not?
#gsettings set org.gnome.settings-daemon.plugins.media-keys volume-mute "['<Alt>m']"
#gsettings set org.gnome.settings-daemon.plugins.media-keys volume-up "['<Alt>k']"
#gsettings set org.gnome.settings-daemon.plugins.media-keys volume-down "['<Alt>j']"
# TODO play / pause?

# TODO aliases for switching between a more responsive (as below) and less
# responsive set of settings, for going easy w/ the repetetive strain...
# This link suggests that repeat-interval should probably be positive,
# but otherwise can be as small as I want (still an int type).
# http://blog.rodolfocarvalho.net/2017/01/configuring-keyboard-repeat-rate-on.html
# (default=30ms)
#gsettings set org.gnome.desktop.peripherals.keyboard repeat-interval 15
# (default=500ms, checked from `gsettings reset ...`)
#gsettings set org.gnome.desktop.peripherals.keyboard delay 280

# NOTE: ctrl_q_hotkey.sh uses xdotool, which only works in X11 sessions (the default
# session is Wayland now)
#set_keyboard_shortcuts.py 'VIM term close + passthrough' 'ctrl_q_hotkey.sh' '<Control>Q'

# Top bar clock like "Fri Nov 13  6:05 PM" (see wishlist_features.md for the custom format
# the Unity clock had)
gsettings set org.gnome.desktop.interface clock-show-date true
gsettings set org.gnome.desktop.interface clock-show-weekday true
gsettings set org.gnome.desktop.interface clock-format '12h'
