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
sudo apt-get install -y build-essential cmake python3-dev

# TODO modify so vim doesn't show output here if possible (or not interactive)
# (so vim doesn't spit out "Warning: Output is not to a terminal" when called
# from `install`
# https://github.com/junegunn/vim-plug
# Installs missing plugins and updates the rest, then (the `!`) runs the 'do' hooks in vimrc
# of all of them, changed or not: building YouCompleteMe (install.py --clangd-completer),
# and downloading what markdown-preview.nvim and vim-doge run. vim-plug runs synchronously
# while Vim is starting, so this all finishes before +qall.
vim +'PlugUpdate!' +qall
