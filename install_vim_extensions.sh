#!/usr/bin/env bash

# TODO maybe also allow overriding this if called from main install.conf.yaml
# instructions, and the install script that does this precedes
# https://stackoverflow.com/questions/1885525
read -p "'sudo apt-get update' before installing YouCompleteMe deps? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo apt-get update
fi

read -p "'update all git submodules? (y/n)" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    git submodule update --init --recursive
fi

# TODO test this works on fresh 16.04, 18.04, and WSL
# (see old version of this file when it was named ycm_deps.sh if those
# deps work better than these ones)
sudo apt-get install -y build-essential cmake python3-dev

# TODO modify so vim doesn't show output here if possible (or not interactive)
# (so vim doesn't spit out "Warning: Output is not to a terminal" when called
# from `install`
# https://github.com/VundleVim/Vundle.vim
vim +PluginInstall +qall

# Important it happens after the vim call above, because that's when Vundle will
# download the YouCompleteMe source.
# Just assuming script called from `dotfiles` root directory for now.
cd vim/bundle/YouCompleteMe

# Using this option rather than --all b/c not interested in other languages
# explicitly mentioned in --help output, and some can cause compilation to fail,
# like C#. Python support still works for me in WSL w/ these options.
# Maybe just call with no options?
./install.py --clang-completer --quiet

