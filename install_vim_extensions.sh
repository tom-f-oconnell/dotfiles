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
# https://github.com/VundleVim/Vundle.vim
# TODO test this works on fresh system. seems to work though.
vim +PluginInstall +"mkdp#util#install" +"doge#install" +qall

# TODO maybe test if YCM is set up and don't install this if it is? maybe w/ option to
# force reinstall / at least printing how to do so?
# could try parsing output of :YCMDebugInfo?

# Important it happens after the vim call above, because that's when Vundle will
# download the YouCompleteMe source.
# Just assuming script called from `dotfiles` root directory for now.
cd vim/bundle/YouCompleteMe

# Using this option rather than --all b/c not interested in other languages
# explicitly mentioned in --help output, and some can cause compilation to fail,
# like C#. Python support still works for me in WSL w/ these options.
# Maybe just call with no options?
./install.py --clang-completer --quiet

