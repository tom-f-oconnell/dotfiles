#!/usr/bin/env bash

# This script exists because the change I made to ~/.gitconfig (in the same
# commit that adds this script) causes conflicts on push for repos that already
# have their local configs set.

# TODO test behavior if not already set

# TODO maybe test the values are equal to the ones in ~/.gitconfig? or print
# all + grep? -> confirm?

# If there was already a section for these in ./.git/config, there will probably
# still be the [branch "master"] line that used to contain the lines we unset,
# but that shouldn't be an issue.
git config --unset branch.master.remote
git config --unset branch.master.merge

