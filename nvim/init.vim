" Neovim config (~/.config/nvim): reuses ~/.vimrc and ~/.vim, which check has('nvim')
" where Neovim needs something different, plus the Neovim-only settings below.

set runtimepath^=~/.vim runtimepath+=~/.vim/after
let &packpath = &runtimepath

" Python 3 provider (needed by YouCompleteMe), in a venv w/ pynvim (from
" install_neovim.sh). Not whatever python3 is first in PATH, which may be a conda env's.
let g:python3_host_prog = expand('~/.local/share/nvim/venv/bin/python')
" Unused providers (otherwise searched for at startup, and warned about in :checkhealth).
let g:loaded_node_provider = 0
let g:loaded_perl_provider = 0
let g:loaded_ruby_provider = 0

source ~/.vimrc

" taghelper.vim's Python (for the current function in the statusline) calls
" vim.bindeval(), which only Vim's Python API has (for &verbose and &rtp, as bytes).
python3 << EOF
import vim
if not hasattr(vim, 'bindeval'):
    def _bindeval(expr):
        # Not vim.eval(), which (like Vim's) turns numbers into strings.
        value = vim.api.eval(expr)
        return value.encode() if isinstance(value, str) else value
    vim.bindeval = _bindeval
EOF

" Undo Neovim defaults that differ from Vim's (w/o defaults.vim, which a vimrc disables),
" to behave like Vim w/ the same vimrc.
set nohlsearch
" vim-indexed-search already shows the search count.
set shortmess+=S
" Y yanks the whole line, rather than to the end of it.
silent! nunmap Y

" Swap files next to the edited file (Vim's default), rather than under
" ~/.local/state/nvim/swap, so each editor warns about files open in the other.
set directory=.,~/tmp,/var/tmp,/tmp
" Prompt (as Vim does) when a file is already open in another Vim / Neovim, rather than
" Neovim's default of just editing it anyway, w/ a W325 warning.
autocmd! nvim.swapfile

lua require('markdown_render')
