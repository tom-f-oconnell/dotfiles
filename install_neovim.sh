#!/usr/bin/env bash
# Installs a Neovim nightly build under ~/.local/opt/nvim (linked from ~/.local/bin/nvim),
# and a venv with pynvim for Neovim's Python 3 provider (which YouCompleteMe needs).
#
# Nightly rather than a release, for conceal-aware 'wrap' (concealed text not counting
# toward line width), which render-markdown.nvim relies on and no release has yet (it
# should be in 0.13). Pass -u to replace an existing install with the latest nightly.
set -euo pipefail

INSTALL_DIR=~/.local/opt/nvim
VENV=~/.local/share/nvim/venv
ASSET=nvim-linux-$(uname -m).tar.gz
URL=https://github.com/neovim/neovim/releases/download/nightly/$ASSET

update=0
if [ "${1:-}" = "-u" ]; then
    update=1
fi

if [ "$update" = 1 ] || [ ! -x "$INSTALL_DIR/bin/nvim" ]; then
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    echo "Downloading $URL"
    curl -fL --progress-bar -o "$tmp/$ASSET" "$URL"
    mkdir "$tmp/extracted"
    tar -C "$tmp/extracted" -xzf "$tmp/$ASSET"
    rm -rf "$INSTALL_DIR"
    mkdir -p "$(dirname "$INSTALL_DIR")"
    mv "$tmp/extracted/${ASSET%.tar.gz}" "$INSTALL_DIR"
fi
mkdir -p ~/.local/bin
ln -sfn "$INSTALL_DIR/bin/nvim" ~/.local/bin/nvim
~/.local/bin/nvim --version | head -1

# System Python (not whatever python3 is first in PATH, e.g. from conda), matching the
# one YouCompleteMe is built with (see install_vim_extensions.sh).
if [ ! -x "$VENV/bin/python" ]; then
    /usr/bin/python3 -m venv "$VENV"
fi
"$VENV/bin/pip" install --quiet --upgrade pynvim

# Installs plugins only Neovim uses (see `Cond(has('nvim'))` in vimrc), if missing.
~/.local/bin/nvim --headless +PlugInstall +qall
