#!/usr/bin/env bash

declare -a autoedited=(
"inkscape/preferences.xml"
"LibreCAD/LibreCAD.conf"
)
for f in "${autoedited[@]}"
do
    echo "git update-index --skip-worktree $f"
    # https://stackoverflow.com/questions/13630849
    git update-index --skip-worktree $f
done
