#!/usr/bin/env bash

mkdir -p ~/src
cd ~/src

if [ ! -d ~/src/scripts ] ; then
    git clone git@github.com:tom-f-oconnell/scripts.git
else
    cd scripts
    git pull --rebase || (exit 0)
fi
