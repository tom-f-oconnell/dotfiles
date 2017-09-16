
autocmd Filetype python setlocal expandtab tabstop=4 shiftwidth=4

autocmd! BufNewFile,BufRead *.ino,*.pde setlocal ft=arduino
autocmd Filetype arduino setlocal expandtab tabstop=2 shiftwidth=2

autocmd! BufNewFile,BufRead *.launch setlocal ft=launch
autocmd Filetype launch setlocal expandtab tabstop=2 shiftwidth=2

set backspace=indent,eol,start
