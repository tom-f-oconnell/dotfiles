
" can this be reversed? not sure I mind?
set nocompatible
filetype off
" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim

" TODO maybe detect location of dotfiles dir on install and then replace 
" both this + an absolute path in vundle#begin call with that?
" Windows only
" (test visualbell OK on Linux or special case in VIM / deployment)
set visualbell

" TODO TODO configure s.t. vundle doesn't add stuff to my dotfiles repo
" in a way that would either be confusing or interfere with anything.
" this probably means either telling git to ignore some dir or have vundle
" always install to some other path.
call vundle#begin()
" alternatively, pass a path where Vundle should install plugins
"call vundle#begin('~/some/path/here')

" TODO i didn't seem to need to specify paths explicitly w/ vundle on ubuntu...
" is that cause pathogen was actually managing YouCompleteMe despite it being
" under 'bundle'?
" how can i get it to work on WSL and ubuntu in a way that doesn't require
" explicitly hardcoding in each plugin??

" TODO test paths above (rtp + begin) allow this to work in my dotbot repo
" (don't want to have to manually download this, and want vundle to be able
" to update / manage itself as "required" too. test!)
" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'

" TODO test vundle can always compile this (ideally on WSL too)
" if it can't, may want to handle this one manually.
Plugin 'Valloric/YouCompleteMe'

" For "aligning text" figure out how to use.
Plugin 'godlygeek/tabular'

" Shows indicator in left col of which lines differ from HEAD, etc.
" TODO delete if i don't like it. haven't used much.
Plugin 'airblade/vim-gitgutter'

" TODO try dense-analysis/ale for linting, but drop if annoying.
" https://www.reddit.com/r/vim/comments/c2f1bl top comment says he prefers
" offline 'black' b/c ALE is annoying.

" I think I used to use this to display this in the status bar. Try again?
Plugin 'tyru/current-func-info.vim'

" TODO more modern version of this? some other site seemed to have something
" that highlighted the opening and closing whatever; did it close them too?
Plugin 'Raimondi/delimitMate'

Plugin 'plasticboy/vim-markdown'
Plugin 'lervag/vimtex'

" All of your Plugins must be added before the following line
call vundle#end()
" To ignore plugin indent changes, instead use:
"filetype plugin on

" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just 
"                     :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to
"                     auto-approve removal

" see :h vundle for more details or wiki for FAQ

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
" TODO should this actually be 79 to follow PEP8? maybe set in python files?
" does this / PEP8 count newline?
set textwidth=80

" So that *I* can manually keep lines to 80 characters
" Highlights extra characters (beyong 80) in red.
" I might prefer this approach over the ever-shown column, commented below
" Relative to textwidth (tw) because it should autowrap (hard, w/ newline)
" there.
" TODO make this relative to current textwidth (maybe one of two diff colors
" if text is between current tw and 80, one color for either direction?)
" TODO make exception for URLs if possible. i often don't want to break those.
au BufWinEnter * let w:m2=matchadd('ErrorMsg', '\%>80v.\+', -1)

" TODO maybe if i would get "No identifier under cursor" error (trying to enter
" insert mode when caps lock is on), switch caps lock off and enter insert mode?
" TODO or always display some indicator that caps lock is on (if possible)?

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
au Filetype python setlocal expandtab tabstop=8 shiftwidth=4 softtabstop=4
" not sure if the VIM I generally use is new enough to use the 'shiftwidth()'
" format
" TODO do something similar for other languages.
" some guy was saying "filetype plugin indent on" worked for him, but how? I
" have that up top, so it must be overridden?
let g:pyindent_continue = '&shiftwidth'

" sets new python files to executable by default
" TODO tabs to spaces in vimrc
au BufWritePre *.py if !filereadable(expand('%')) | 
	\let b:is_new = 1 | endif
au BufWritePost *.py if get(b:, 'is_new', 0) | 
	\silent execute '!chmod +x %' | endif

" TODO also map in insert mode?
" see http://vim.wikia.com/wiki/Python_-_check_syntax_and_run_script
" first <cr> necessary?
" also see:
" https://stackoverflow.com/questions/18948491/running-python-code-in-vim
" TODO maybe make it so it shows up in window / split at bottom, w/ hotkey
" to exit that window?
" TODO open file -> (in template) add some command -> press F5 in insert (<F5>
" just inserted)
" TODO open file -> (in template) add some command -> press F5 in command before
" writing file -> command not apparently run, but pressing :w yields "Warning:
" Mode of file "hi.py" has changed since editing started." fix
" TODO why will this not seem to run if the python file is not executable?
" i can normally call a non-executable python script by passing it to the
" interpreter...
" TODO see
" https://stackoverflow.com/questions/12030965/
" change-the-mapping-of-f5-on-the-basis-of-specific-file-type
" TODO maybe use shebang when possible, since that could specify python
" version (now i'm exclusively using shebang, and incidentally also probably
" relying on file being executable
au BufRead *.py nmap <F5> :w<cr>:!./%<cr>
au BufRead *.py imap <F5> <Esc>:w<cr>!./%<cr>
"au BufRead *.py nmap <F5> :w<cr>:!python %<cr>
"au BufRead *.py imap <F5> <Esc>:w<cr>!python %<cr>

" TODO how to define these kind of options together, for reuse with diff
" filetypes?
au BufNewFile *.py 0r ~/.vim/skel.py
au BufNewFile *.py normal G
au BufNewFile *.py startinsert

au Filetype html setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=2

au Filetype sh setlocal expandtab tabstop=4 shiftwidth=4
au BufWritePre *.sh if !filereadable(expand('%')) | 
	\let b:is_new = 1 | endif
" TODO why can't i just do this directly in above?
au BufWritePost *.sh if get(b:, 'is_new', 0) | 
	\silent execute '!chmod +x %' | endif
au BufRead *.sh nmap <F5> :w<cr>:!bash %<cr>
au BufRead *.sh imap <F5> <Esc>:w<cr>!bash %<cr>
au BufNewFile *.sh 0r ~/.vim/skel.sh
au BufNewFile *.sh normal G
au BufNewFile *.sh startinsert

" TODO what does au! do again? might be undesirable / affect order
" TODO move cursor to inside setup and enter insert
au! BufNewFile,BufRead *.ino,*.pde setlocal ft=arduino
au Filetype arduino setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=2
" TODO augroup on (ft?) to only specify *.ino,*.pde once, and similarly for
" other languages?
" this had to come after the aus
au BufNewFile *.ino,*.pde 0r ~/.vim/skel.ino

" for tab delimiting keywords.txt, as supposed to
au! BufNewFile,BufRead keywords.txt setlocal ft=arduino_keywords_txt
au Filetype arduino_keywords_txt setlocal shiftwidth=8 noexpandtab softtabstop=0

au! BufNewFile,BufRead *.launch setlocal ft=launch
au Filetype launch setlocal expandtab tabstop=2 shiftwidth=2

set spellfile=$HOME/src/dotfiles/vim/spell/spellfile.utf-8.add
au! BufNewFile,BufRead *.md setlocal ft=markdown
au Filetype markdown setlocal expandtab tabstop=3 shiftwidth=3
au Filetype markdown setlocal spell spelllang=en_us

" To exclude spellcheck from certain .txt files
au! BufNewFile,BufRead *requirements.txt setlocal ft=special_txt
" TODO why do I explicitly need to set this? don't for source files for
" instance... (still inheriting some text / default properties that are set
" differently be default for autodetected code files?)
au Filetype special_txt setlocal nospell

" Trying to include some txt settings for my usual habits of making lots of
" nested bulleted lists, with indents at one level, often with - as prefix.
" TODO make it more like markdown?
" TODO Filetype and FileType both valid?
au Filetype text setlocal expandtab tabstop=1 shiftwidth=1
au Filetype text setlocal spell spelllang=en_us

au Filetype yaml setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=0

" TODO set vs setlocal? BufFilePost in others too?
au BufNewFile,BufRead,BufFilePost *.cir setlocal filetype=spice

set backspace=indent,eol,start
set autoindent

" TODO what was the purpose of this again?
if has("au")
  filetype plugin indent on
endif

" To turn off the preview window in YouCompleteMe (or other completions)
set completeopt-=preview

" For easier configuring on a file by file basis with the # vim: options line
" set modeline

" Trying the default spell suggestions out. May turn off.
" Too busy. Want a way to spellcheck quickly though, and then turn it off.
" set spell spelllang=en_us

" Commented since I don't use these anymore, and might be better use out of
" these F keys
" For Overtone / Clojure interfacing
"map <F2> :Eval<ENTER>
"map <F3> :Connect<ENTER>1<ENTER><ENTER>

" vim-markdown configuration
let g:vim_markdown_folding_disabled = 1

" TODO automatically delete through comment when deleting from middle of comment
" block? or just use gq?

" TODO shortcut/macro to insert date

" show the current function name on statusline. works for at least Python.
" TODO possible to work in whitespace / empty lines in function too? (doesn't
" now)
" TODO fix false positives (e.g. in choice_analysis, in top-level code after
" press, the status bar says we are still in the function press)
" TODO uncomment after adding default info back (at least line #)
" https://unix.stackexchange.com/questions/224771/what-is-the-format-of-the-default-statusline?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
"let &statusline .= ' %{cfi#format("%s", "")}'
" TODO just display func name in another color? (or flank w/ symbols like
" ex?)
"hi statusline ctermfg=5 ctermbg=0

" should make it so the statusline is always displayed
set laststatus=2

" should make mouse scrolling work inside tmux
" enables mouse (just scrolling? selection, etc?) in [a]ll modes
set mouse=a


" Custom functions on the 'Leader' keyboard
let mapleader = ","
nnoremap <leader>b oimport ipdb; ipdb.set_trace()<Esc>

noremap <F12> <Esc>:syntax sync fromstart<CR>
inoremap <F12> <C-o>:syntax sync fromstart<CR>

" TODO are my .sh tab settings correct?
" (apparently, some spec says to use tabs?)


set pastetoggle=<F2>


" The link says this should enable/disable paste mode automatically, for
" certain types of pasting.
" https://coderwall.com/p/if9mda/automatically-set-paste-mode-in-vim-when-pasting-in-insert-mode
" See link for code to also make this work in tmux, if I feel the need.

" TODO TODO any reason i can't / shouldn't also have this enable insert mode?
" (i guess i'd have to change it to not be "inoremap" then, since that
" specifies an insert mode binding

let &t_SI .= "\<Esc>[?2004h"
let &t_EI .= "\<Esc>[?2004l"
inoremap <special> <expr> <Esc>[200~ XTermPasteBegin()

" I'm not really sure how this is also setting nopaste at the end, but I tested
" it, and it is.
function! XTermPasteBegin()
  set pastetoggle=<Esc>[201~
  set paste
  return ""
endfunction

