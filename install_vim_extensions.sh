#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

# YouCompleteMe needs build-essential, cmake and python3-dev to build, which
# ~/src/scripts/movein.sh installs (it runs before this in `install`)

# -es (silent Ex mode, which needs the -u to still read vimrc) so Vim doesn't warn that
# "Output is not to a terminal" and wait at "Press ENTER" prompts when called from
# `install`, which doesn't show this step's stdout.
# https://github.com/junegunn/vim-plug
# Installs missing plugins and updates the rest, then (the `!`) runs the 'do' hooks in vimrc
# of all of them, changed or not: building YouCompleteMe (install.py --clangd-completer),
# and downloading what markdown-preview.nvim and vim-doge run. vim-plug runs synchronously
# while Vim is starting, so this all finishes before +qall.
vim -es -u ~/.vimrc -i NONE +'PlugUpdate!' +qall
