#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

# TODO also have update / install of YCM deps only happen if YCM isn't set up correctly

# TODO maybe also allow overriding this if called from main install.conf.yaml
# instructions, and the install script that does this precedes
# https://stackoverflow.com/questions/1885525
read -p "'sudo apt-get update' before installing YouCompleteMe deps? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo apt-get update
fi

# TODO test this works on fresh 20.04, and WSL
# YouCompleteMe also needs build-essential and cmake to build, which
# ~/src/scripts/movein.sh installs (it runs before this in `install`)
sudo apt-get install -y python3-dev

# -es (silent Ex mode, which needs the -u to still read vimrc) so Vim doesn't warn that
# "Output is not to a terminal" and wait at "Press ENTER" prompts when called from
# `install`, which doesn't show this step's stdout.
# https://github.com/junegunn/vim-plug
# Installs missing plugins and updates the rest, then (the `!`) runs the 'do' hooks in vimrc
# of all of them, changed or not: building YouCompleteMe (install.py --clangd-completer),
# and downloading what markdown-preview.nvim and vim-doge run. vim-plug runs synchronously
# while Vim is starting, so this all finishes before +qall.
vim -es -u ~/.vimrc -i NONE +'PlugUpdate!' +qall
