
" Not sure exactly what pathogen does?
execute pathogen#infect()

syntax on
filetype plugin indent on

" To alleviate stress from hitting the escape key
inoremap jj <ESC>
inoremap kk <ESC>

" Maps W to save as well, since my computers accidentally capitalize W in vim often
command W w

autocmd Filetype python setlocal expandtab tabstop=4 shiftwidth=4

autocmd! BufNewFile,BufRead *.ino,*.pde setlocal ft=arduino
autocmd Filetype arduino setlocal expandtab tabstop=2 shiftwidth=2

autocmd! BufNewFile,BufRead *.launch setlocal ft=launch
autocmd Filetype launch setlocal expandtab tabstop=2 shiftwidth=2

autocmd! BufNewFile,BufRead *.md setlocal ft=markdown
autocmd Filetype markdown setlocal expandtab tabstop=3 shiftwidth=3

set backspace=indent,eol,start
set autoindent

" TODO what was the purpose of this again?
if has("autocmd")
  filetype plugin indent on
endif

" To turn off the preview window in YouCompleteMe (or other completions)
set completeopt-=preview

" For easier configuring on a file by file basis with the # vim: options line
" set modeline

" Trying the default spell suggestions out. May turn off.
" Too busy. Want a way to spellcheck quickly though, and then turn it off.
" set spell spelllang=en_us

" For Overtone / Clojure interfacing
map <F2> :Eval<ENTER>
map <F3> :Connect<ENTER>1<ENTER><ENTER>

" vim-markdown configuration
let g:vim_markdown_folding_disabled = 1
