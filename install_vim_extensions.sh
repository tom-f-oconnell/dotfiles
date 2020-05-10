#!/usr/bin/env bash

# TODO maybe also allow overriding this if called from main install.conf.yaml
# instructions, and the install script that does this precedes
# TODO test
# https://stackoverflow.com/questions/1885525
read -p "'sudo apt-get update' before installing YouCompleteMe deps?" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo apt-get update
fi

# TODO did it matter that this script was called w/ sudo in dotbot and no sudo
# in here?

# TODO test this works on fresh 16.04, 18.04, and WSL
# (see old version of this file when it was named ycm_deps.sh if those
# deps work better than these ones)
sudo apt-get install -y build-essential cmake python3-dev

# https://github.com/VundleVim/Vundle.vim
vim +PluginInstall +qall

# Important it happens after the vim call above, because that's when Vundle will
# download the YouCompleteMe source.
# Just assuming script called from `dotfiles` root directory for now.
# TODO check that this cd doesn't break calling script (it probably shouldn't)
# (doing it this way b/c i feel compilation script may expect that)
cd vim/bundle/YouCompleteMe

# TODO maybe use --quiet option?

# Using this option rather than --all b/c not interested in other languages
# explicitly mentioned in --help output, and some can cause compilation to fail,
# like C#. Doesn't seem like it should affect Python support?
# Maybe just call with no options?
./install.py --clang-completer

