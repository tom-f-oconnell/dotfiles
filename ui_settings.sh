#!/usr/bin/env bash

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
gsettings set org.gnome.settings-daemon.plugins.media-keys volume-mute '<Alt>m'
gsettings set org.gnome.settings-daemon.plugins.media-keys volume-up '<Alt>k'
gsettings set org.gnome.settings-daemon.plugins.media-keys volume-down '<Alt>j'
# TODO play / pause?
