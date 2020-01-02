#!/usr/bin/env bash

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

# default 2x2 workspaces
gsettings set org.compiz.core:/org/compiz/profiles/unity/plugins/core/ hsize 2
gsettings set org.compiz.core:/org/compiz/profiles/unity/plugins/core/ vsize 2

gsettings set com.canonical.Unity.Launcher favorites "['application://org.gnome.Nautilus.desktop', 'application://firefox.desktop', 'application://rhythmbox.desktop', 'application://libreoffice-calc.desktop', 'application://libreoffice-writer.desktop', 'unity://running-apps', 'application://gnome-terminal.desktop', 'unity://expo-icon', 'unity://devices']"

# keybindings, to have consistent keys for volume control across all computers
# unfortunately, it seems ubuntu (at least up through 16.04) does not really
# support multiple keybindings to same function. (would like to support any
# native volume control keys as well)
# TODO maybe i could detect whether there are native keyboard controls on the
# particular computer, and only set these if not?
gsettings set org.gnome.settings-daemon.plugins.media-keys volume-mute '<Alt>m'
gsettings set org.gnome.settings-daemon.plugins.media-keys volume-up '<Alt>k'
gsettings set org.gnome.settings-daemon.plugins.media-keys volume-down '<Alt>j'
# TODO play / pause?


# This link suggests that repeat-interval should probably be positive,
# but otherwise can be as small as I want (still an int type).
# http://blog.rodolfocarvalho.net/2017/01/configuring-keyboard-repeat-rate-on.html
# (default=30ms)
gsettings set org.gnome.desktop.peripherals.keyboard repeat-interval 15
# (default=500ms, checked from `gsettings reset ...`)
gsettings set org.gnome.desktop.peripherals.keyboard delay 280

# In the meantime, Displays->Sticky edges->Off
# This is so the mouse doesn't get stuck on the boundary between monitors.
# Not sure if I do want the default "sticky edges" in some cases though...
# is there some way to adjust the threshold mouse velocity here?
# (does not seem to be the relevant parameter when using Unity, see:
# https://unix.stackexchange.com/questions/440334 )
# https://askubuntu.com/questions/462629
#gsettings set org.gnome.shell.overrides edge-tiling false
# full path for this "relocatable schema" taken from here:
# https://askubuntu.com/questions/290159
# still not sure what (if any) general procedure exists for finding paths to
# relocatable schemas...
gsettings set org.compiz.unityshell:/org/compiz/profiles/unity/plugins/unityshell/overcome-pressure 1
# there is also: org.gnome.metacity edge-tiling, but this doesn't seem like it
# is that likely to influence unity settings and it is also not changed after
# disabling sticky edges in gui settings

# TODO add an answer to https://askubuntu.com/questions/109338 , to reflect the
# solution i came up w/ to disable sticky edges. nothing there works w/ 16.04
# from cli


