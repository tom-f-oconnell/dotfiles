
" Not sure exactly what pathogen does?
execute pathogen#infect()

syntax on
filetype plugin indent on

" Prevents vim from breaking in the middle of words, the default.
" May prefer to break at space + punctuation? See breakat option.
" TODO is this even worth it, if hard-breaking at 80 (w/ tw)?
set linebreak

" TODO maybe remap gj and gk to j / k (works within single lines, I think?)

" Allows yanking and pasting between terminals on the same host.
" (depends on :version having +xterm_clipboard)
" TODO looks like i need to compile my own vim if I want this feature
" see https://stackoverflow.com/questions/37079424/\
" ubuntu-16-04-lts-cant-enable-xterm-clipboard-in-vim
" TODO could this be done using other registers *(?) w/o compile flag?
set clipboard+=unnamed

" TODO i think i might want to prevent 'x' from going in to buffer? way to
" prevent? maybe i actually would want to keep it, in case i get more
" proficient with vim?

" To alleviate stress from hitting the escape key
inoremap jj <ESC>
inoremap kk <ESC>

" Maps W to save as well, since my computers accidentally capitalize W in VIM
" sometimes. not sure why they (rarely) do / did?
command W w

" TODO would be nice to have some (language specific?) automation that follows
" some the the practices I normally take to break up lines

" Cutoff, beyond which, vim will wrap, breaking with a new-line
set textwidth=80

" So that *I* can manually keep lines to 80 characters
" Highlights extra characters (beyong 80) in red.
" I might prefer this approach over the ever-shown column, commented below
" Relative to textwidth (tw) because it should autowrap (hard, w/ newline)
" there.
" TODO make this relative to current textwidth (maybe one of two diff colors
" if text is between current tw and 80, one color for either direction?)
au BufWinEnter * let w:m2=matchadd('ErrorMsg', '\%>80v.\+', -1)

" set colorcolumn=80
" To change the color of the colorcolumn use :highlight ColorColumn, e.g.
" highlight ColorColumn ctermbg=lightgrey guibg=lightgrey

" TODO do i want these defaults or not?
" Linux kernel uses 8, but I like sticking to <=80 chars per line, and
" (even small) multiples of 8 cut into that pretty quick.
set tabstop=4
set softtabstop=4
set shiftwidth=4
set expandtab

" https://stackoverflow.com/questions/16047521/\
" make-vim-indent-c-preprocessor-directives-the-same-as-other-statements
" These cinkeys statements should be equivalent:
"set cinkeys=0{,0},0),:,!^F,o,O,e
set cinkeys-=0#
set cinoptions+=#1s

" TODO what is tabstop exactly? i had it at 4... do i need to reformat?
autocmd Filetype python setlocal expandtab tabstop=8 shiftwidth=4 softtabstop=4
" not sure if the VIM I generally use is new enough to use the 'shiftwidth()'
" format
" TODO do something similar for other languages.
" some guy was saying "filetype plugin indent on" worked for him, but how? I
" have that up top, so it must be overridden?
let g:pyindent_continue = '&shiftwidth'

autocmd Filetype sh setlocal expandtab tabstop=4 shiftwidth=4

autocmd! BufNewFile,BufRead *.ino,*.pde setlocal ft=arduino
autocmd Filetype arduino setlocal expandtab tabstop=2 shiftwidth=2

autocmd! BufNewFile,BufRead *.launch setlocal ft=launch
autocmd Filetype launch setlocal expandtab tabstop=2 shiftwidth=2

autocmd! BufNewFile,BufRead *.md setlocal ft=markdown
autocmd Filetype markdown setlocal expandtab tabstop=3 shiftwidth=3

" Trying to include some txt settings for my usual habits of making lots of
" nested bulleted lists, with indents at one level, often with - as prefix.
" TODO make it more like markdown?
autocmd Filetype text setlocal expandtab tabstop=1 shiftwidth=1

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
