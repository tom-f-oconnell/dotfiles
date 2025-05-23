" TODO TODO hotkey to open current file in github (or to make github permalink to
" selected lines?) (-> copy to clipboard) (similar to gw alias, maybe make hokey keys as
" similar as possible)
" TODO any way to I->help info for token to pull up web pages for documentation
" (at least for major stuff in python, like stdlib/numpy/pandas/xarray/seaborn)
" TODO are documentation sites generally easy to find from something in source code?
" what about finding the specific function page? sphinx do something that helps with
" that?

" TODO try out default seems settings and see if there is any behavior i would like to
" preserve, as in: https://vi.stackexchange.com/questions/22944

" TODO TODO figure out (editing this is needed) how to use the visual selection
" yank i often use to copy/paste multiple lines in vim to copy lines to the
" system clipboard (and maybe warn w/ appropriate error if i can't, b/c vim /
" system settings?)
" TODO for bonus points, maybe delete any asserts in the yanked it?
" (or another hotkey to do so) (for executing stuff in debugger, which is also
" the main goal above)

" TODO make vim not start new lines when editing ~/.vimrc w/ comment
" (if current edited line was a comment, this is the current behavior)

" TODO TODO TODO setuptools setup.py skeleton

" TODO maybe readme.md skeleton (if there's much of a standard format, like:
" deps, install, usage, etc)?

" TODO TODO if we would open one of those tmp files (happens when no arg?)
" ...just don't open it (need to handle in bash, before vim?)

" TODO possible to make shortcuts for searching only comments / code lines??
" could occasionally be quite useful in files with lots of commented code
" or extensive doc strings (googling didn't turn up much)

" Note: if I decide to make my own vim plugin, definitely read this
" http://vimcasts.org/blog/2014/02/follow-my-leader before implementing any type
" of hotkeys.

" can this be reversed? not sure I mind?
set nocompatible
" TODO TODO why does commenting this seem to screw up csv plugin initialization (despite
" errors showing indicating i should probably not have this...)
" TODO why did i have this off? (do think i turn it on later tho...)
filetype off
" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim

" TODO maybe detect location of dotfiles dir on install and then replace
" both this + an absolute path in vundle#begin call with that?
" Windows only
" (test visualbell OK on Linux or special case in VIM / deployment)
" set visualbell
" The above visualbell setting was actually super annoying, with all the
" flashes.
" TODO test that the below still removes audible bell on windows
set belloff=all

" TODO TODO vundle hasn't been updated in several years... maybe i should use
" some other VIM package manager

" TODO TODO why was this not changing which python is actually used?
"let g:ycm_server_python_interpreter = '/usr/bin/python3.6'

" TODO TODO TODO possible to get YCM to recognize unchanged .ycm_extra_conf.py files
" don't need further confirmation after first OK (similar to how direnv works before
" requiring another `direnv allow`). modify YCM?
" see also g:ycm_confirm_extra_conf and g:ycm_extra_conf_globlist
" Technically risky, but who is really going to use this as a vector...
let g:ycm_confirm_extra_conf = 0

" TODO try to fix how this currently doesn't open in read-only mode if file is being
" edited by another vim process (or prompt, like splitting normally would)
" Default is 'same-buffer'
let g:ycm_goto_buffer_command = 'split-or-existing-window'

" Default: 1000
let g:ycm_disable_for_files_larger_than_kb = 10000

" TODO TODO TODO if ever gonna use YCM RefactorRename feature, might need to see the
" quickfix window for that, and i think this function also applies there. possible to
" tell which is being called?
" TODO maybe just delete this customization fn and call :ccl in <leader>G command?
" (or leave customization and call :copen in a RefactorRename shortcut?)
" TODO TODO TODO why does RefactorRename take SOOOO long. i have only ever had it time
" out on me, both for cases where the name actually went across files and cases where it
" didn't. see: https://gitter.im/Valloric/YouCompleteMe?at=5f1ca96c65895258e89ea2f9 for
" some discussion.
" `function!` (vs `function`) means to overwrite the function if it exists.
function! s:CustomizeYcmQuickFixWindow()
  " Close quickfix window (can still search through w/ :cn/:cp or my <leader>[N/n]
  " bindings to vim-qf equivalent commands)
  ccl
endfunction

autocmd User YcmQuickFixOpened call s:CustomizeYcmQuickFixWindow()

" TODO would `let g:ycm_disable_signature_help = 1` and disabling (how? doesn't seem
" doc'd) `g:ycm_auto_hover` both disable the popups i find annoying when shown for
" python stdlib stuff?

" TODO TODO try moving to using either vim-plug or builtin package management.
" see: https://github.com/VundleVim/Vundle.vim/issues/955 for reasons Vundle dev hasn't
" been active for a while

" TODO TODO configure s.t. vundle doesn't add stuff to my dotfiles repo
" in a way that would either be confusing or interfere with anything.
" this probably means either telling git to ignore some dir or have vundle
" always install to some other path.

let g:csv_bind_B = 1

call vundle#begin()
" alternatively, pass a path where Vundle should install plugins
"call vundle#begin('~/some/path/here')

" TODO i didn't seem to need to specify paths explicitly w/ vundle on ubuntu...
" is that cause pathogen was actually managing YouCompleteMe despite it being
" under 'bundle'?
" how can i get it to work on WSL and ubuntu in a way that doesn't require
" explicitly hardcoding in each plugin??

" TODO look in to:
" - svermeulen/vim-subversive: quick find replace w/ new 's' motion (maybe can replace
"   some of my hacks for this?)

" TODO test paths above (rtp + begin) allow this to work in my dotbot repo
" (don't want to have to manually download this, and want vundle to be able
" to update / manage itself as "required" too. test!)
" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'

" TODO test vundle can always compile this (ideally on WSL too)
" if it can't, may want to handle this one manually.
Plugin 'Valloric/YouCompleteMe'

Plugin 'dense-analysis/ale'

" For "aligning text" figure out how to use.
Plugin 'godlygeek/tabular'

" Shows indicator in left col of which lines differ from HEAD, etc.
" TODO delete if i don't like it. haven't used much.
Plugin 'airblade/vim-gitgutter'

" TODO what does this do again?
Plugin 'henrik/vim-indexed-search'

" TODO (as per comments below) consider trying to replace w/ tagstack,etc
" (above comment was from when i was still using tyru/current-func-info.vim
" current solution seems to work better w/in python at least, so maybe no need)
Plugin 'mgedmin/taghelper.vim'

" TODO more modern version of this? some other site seemed to have something
" that highlighted the opening and closing whatever; did it close them too?
Plugin 'Raimondi/delimitMate'

Plugin 'plasticboy/vim-markdown'

" Could also try 'instant-markdown/vim-instant-markdown' or
" 'skanehira/preview-markdown.vim'. The latter renders in a VIM split not a browser.
"
" NOTE: requires `:call mkdp#util#install()` after `:PluginInstall`
Plugin 'iamcco/markdown-preview.nvim'

Plugin 'lervag/vimtex'

" Trying mainly for `[f` / `]f` to switch between files, though might want to try
" EditSimilar plugin mentioned in same answer:
" https://stackoverflow.com/questions/25637516
" TODO try to get [f / ]f or similarly short commands to switch between files either of
" same extention or not ignored by git (but include stuff not committed). also default
" to current [f / ]f behavior (adding ignoring of non-[text/code]-like files if
" necessary)
Plugin 'tpope/vim-unimpaired'

Plugin 'ntpeters/vim-better-whitespace'

Plugin 'romainl/vim-qf'

Plugin 'wellle/context.vim'

" TODO TODO figure out how to get this to add a 'Returns: ' section as well (populated
" w/ variables at least for references maybe? might be possible by making a new standard
" from their python one? see their github.)
"
" Adds <leader>d by default (which is what I was using for similar before anyway)
" NOTE: requires `:call doge#install()` after `:PluginInstall`
Plugin 'kkoomen/vim-doge'

Plugin 'preservim/nerdcommenter'

" TODO maybe also try mechatroner/rainbow_csv (similar # of commits / recency, but a bit
" fewer contributors ~2022)
Plugin 'chrisbra/csv.vim'

" lukhio/vim-mapping-conflicts seems useful, but raised a bunch of errors when I tested
" it.

" FastFold strongly recommended in simpylfold docs
Plugin 'Konfekt/FastFold'
Plugin 'tmhedberg/simpylfold'

" TODO test this out:
" https://github.com/python-mode/python-mode
" (just wanted it for folding really, but now i'm planning on using simplyfold for that.
" may still get some use of the more IDE-type features.)
" Plug 'python-mode/python-mode', { 'for': 'python', 'branch': 'develop' }

Plugin 'morhetz/gruvbox'

" All of your Plugins must be added before the following line
call vundle#end()
" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just
"                     :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to
"                     auto-approve removal

" see :h vundle for more details or wiki for FAQ

" https://stackoverflow.com/questions/57014805
function IsWSL()
  if has("unix")
    let lines = readfile("/proc/version")
    if lines[0] =~ "Microsoft"
      return 1
    endif
  endif
  return 0
endfunction

" Defaults to dark w/ my mostly stock gnome-terminal/bash on Ubuntu 20.04, but gets set
" to light in tmux on the same machine, screwing up the colors. See notes in .tmux.conf
" for more details.
set background=dark

" TODO TODO anything possible to make colors more consistent inside / outside of tmux?
" inside tmux, colors seem more muted, as mentioned in link below.
"
" NOTE: still need the outer tmux check for version 3.0a i have on 20.04, at least until
" i figure out how to configure 24-bit color support for it (otherwise vim just has
" white text)
"
" Copied from: https://github.com/morhetz/gruvbox/wiki/Terminal-specific
"
" Use 24-bit (true-color) mode in Vim/Neovim when outside tmux.
" If you're using tmux version 2.2 or later, you can remove the outermost $TMUX check
" and use tmux's 24-bit color support.
" (see http://sunaku.github.io/tmux-24bit-color.html#usage for more information)
if (empty($TMUX))
  if (has("nvim"))
    " For Neovim 0.1.3 and 0.1.4
    " https://github.com/neovim/neovim/pull/2198
    let $NVIM_TUI_ENABLE_TRUE_COLOR=1
  endif
  " For Neovim > 0.1.5 and Vim > patch 7.4.1799
  " https://github.com/vim/vim/commit/61be73bb0f965a895bfb064ea3e55476ac175162
  " Based on Vim patch 7.4.1770 (`guicolors` option)
  " https://github.com/vim/vim/commit/8a633e3427b47286869aa4b96f2bfc1fe65b25cd
  " https://github.com/neovim/neovim/wiki/Following-HEAD#20160511
  if (has("termguicolors"))
    set termguicolors
  endif
endif

" For other themes, see: https://vimcolorschemes.com
"
" Recommended init line (over just `colorscheme gruvbox`) from:
" https://github.com/morhetz/gruvbox/wiki/Installation
autocmd vimenter * ++nested colorscheme gruvbox


" TODO test this isn't triggered in ubuntu
if IsWSL()
    " Type :colorscheme<space> and start pressing tab to see the options.
    colo pablo
endif

" TODO maybe modify to check if current file is empty (want to be able to quit
" accidentally created files quickly, for example from tab completing towards a
" file that shares a prefix with another), rather than checking for
" modification, if that ends up being too disruptive to macro recording (if i
" ever use...)
" Below I remap (normal mode) q to this, and Q to q as in:
" https://stackoverflow.com/questions/10956261
" Q does already enter 'Ex' mode, but I'm pretty sure I didn't want that anyway.
function SaveAndQuit()
    " NOTE: the checking of whether the file is modified is because :wq will change
    " modification time whether or not the file actually changed.
    " Also, I had previously had this fall back to the macro-recording-initiation
    " behavior VIM has by default when 'q' is pressed (if the file wasn't modified),
    " but I found myself just always wanting 'q' to quit, and I still hadn't actually
    " gotten used to using macros for anything.

    " TODO any reason not to replace this w/ `update`? is mod check equiv?
    " https://stackoverflow.com/questions/13107453
    if &mod
        " https://vi.stackexchange.com/questions/2408
        write
    endif

    quit

endfunction

" To settle the confusion I've had as to how to tell whether lines will
" hard-wrap (with appropriate EOL characters inserted).
" Also seems that testing this will require testing more than just one thing,
" hence the function.
" Some references on hard wrapping behavior:
" https://stackoverflow.com/questions/1290285
" http://vimdoc.sourceforge.net/htmldoc/change.html#fo-table
" https://vim.fandom.com/wiki/Automatic_word_wrapping
function WillBreakLines()
    " This '&l:' prefix specifically gets the local option, as setlocal (though
    " I think also works for local variables that are not internal VIM options).
    " The 'l' is not interchangeable with other characters. See :help let for
    " some information.
    " Right hand side is checks if the option string contains the 't' character.
    if &l:tw > 0 && &l:formatoptions =~ 't'
        " Otherwise 0 returned implicitly
        return 1
    endif
endfunction

" So the below method of highlighting the ends of long lines does so with the
" colors I want, rather than the default ~yellow.
" TODO try to find some way of only applying this colorscheme change during this
" command (in case i end up using this highlighting feature in some other
" cases...)
" TODO is 'Search' some kind of builtin [match?] group, of just defined de novo
" here? ':help match' didn't really clarify...
hi Search ctermbg=Red

" TODO make autosplitting of long lines toggle alone with this (via
" formatoptions 't')? (couldn't just WillBreakLines in each call then, before
" deciding to do this at all, as then we couldn't switch it both ways i think)
" or could just leave it s.t. it never does this for if WillBreakLines is true
" for the filetype on load.
" TODO make exception for URLs if possible. i often don't want to break those.
" To toggle highlighting of columns >88 in long lines.
" https://stackoverflow.com/questions/19594119
" TODO possible to make this faster? or force display to update more immediately
" after?
" Starting at 0 so that first call (at end of this file) activates it
" (if appropriate given wrap options)
" Map <leader>h to this below.
let s:activatedh = 0
function ToggleH()
    if !WillBreakLines()
        return
    endif
    if s:activatedh == 0
        let s:activatedh = 1
        " https://vi.stackexchange.com/questions/15955
        execute 'match Search "\%>' . &l:tw . 'v.\+"'
    else
        let s:activatedh = 0
        match none
    endif
endfunction

syntax on

" TODO consolidate w/ lines further below that conditionally sets this...
filetype plugin indent on
" To ignore plugin indent changes, instead use:
"filetype plugin on

" May prefer to break at space + punctuation? See breakat option.
" TODO is this even worth it, if hard-breaking at 88 (w/ tw)?
" TODO TODO figure out how formatoptions 't' works (one letter in option string,
" set/unsettable with 'set formatoptions+=t' / 'set formatoptions-=t') and how
" it interacts with this, wrap/nowrap, and textwidth.
" try to set things up such that only one flag is actually controlling wrap,
" whether that is the 't' formatoption or wrap (so that flag can be used in
" ToggleH, and so default behaviors can be set for different filetypes)
" TODO also figure out where default format options are coming from for
" different filetypes. for example, opening a ft=cpp file i get 'croql',
" but opening ft=launch (now just ft=xml for those), i get 'tcq'.
" NOTE: see the discussion on the same wrapping topic above ToggleH Some people
" are also saying wrap / linebreak (at least by default?) don't actually make
" new lines (inserting appropriate characters), so maybe this never did what I
" wanted.
" Prevents vim from breaking in the middle of words, the default.
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
" Changed to 88 for default black line length to work.
" TODO test whether it is off-by-one wrt black though!
set textwidth=88

" TODO maybe if i would get "No identifier under cursor" error (trying to enter
" insert mode when caps lock is on), switch caps lock off and enter insert mode?
" TODO or always display some indicator that caps lock is on (if possible)?

" set colorcolumn=88
" To change the color of the colorcolumn use :highlight ColorColumn, e.g.
" highlight ColorColumn ctermbg=lightgrey guibg=lightgrey

" TODO do i want these defaults or not?
" Linux kernel uses 8, but I like sticking to <=88 chars per line, and
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

" TODO move filetype specific stuff to ~/vim/ftplugin/<filetype>_mappings.vim files
" (requires `:filetype plugin on`), as https://vi.stackexchange.com/questions/10664

" Since FreeCAD macros are written in Python.
" This is the extension that FreeCAD gives them (at least by default).
au! BufNewFile,BufRead *.FCMacro setlocal ft=python

" TODO how to define these kind of options together, for reuse with diff
" filetypes?
au! BufNewFile *.py 0r ~/.vim/skel.py
" TODO copy name of current directory to default name of package
" TODO move inside install_requires
" TODO TODO uncomment after figuring out how to get this to override the .py
" skeleton above. maybe see: https://vi.stackexchange.com/questions/23248
"au! BufNewFile setup.py 0r ~/.vim/skel.setup.py
au BufNewFile *.py normal G
au BufNewFile *.py startinsert

" TODO what is tabstop exactly? i had it at 4... do i need to reformat?
au Filetype python setlocal expandtab tabstop=8 shiftwidth=4 softtabstop=4
" not sure if the VIM I generally use is new enough to use the 'shiftwidth()'
" format
" TODO do something similar for other languages.
" some guy was saying "filetype plugin indent on" worked for him, but how? I
" have that up top, so it must be overridden?
" TODO TODO what is this line actually doing?
" (not used elsewhere in vimrc... builtin?)
let g:pyindent_continue = '&shiftwidth'

" This link explains difference between au and au! (au=autocmd)
" https://vi.stackexchange.com/questions/19849
" The difference is that au! overwrites previous autocmds in the "group"
" TODO check that all my usages of au vs au! are appropriate though

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
" TODO might want to just use 'map' instead, as there seem to be some modes not
" covered bewteen nmap/imap that i might still want... not 100% sure 'map' is
" what i want either though...
" https://vi.stackexchange.com/questions/2089
" TODO maybe call these inside :term ?
au BufRead *.py nmap <buffer> <F5> :w<cr>:!./%<cr>
au BufRead *.py imap <buffer> <F5> <Esc>:w<cr>!./%<cr>
"au BufRead *.py nmap <buffer> <F5> :w<cr>:!python %<cr>
"au BufRead *.py imap <buffer> <F5> <Esc>:w<cr>!python %<cr>

" sets new python files to executable by default
" TODO tabs to spaces in vimrc
au BufWritePre *.py if !filereadable(expand('%')) |
	\let b:is_new = 1 | endif
" The '| e' adds an :e (edit=reload) command after the chmod, so that VIM
" doesn't warn that the file mode has changed and ask whether you want to
" load it (happens on first save if vim created the file that session)
au BufWritePost *.py if get(b:, 'is_new', 0) |
	\silent execute '!chmod +x %' | e | endif

au Filetype html setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=2

au Filetype sh setlocal expandtab tabstop=4 shiftwidth=4
au BufWritePre *.sh if !filereadable(expand('%')) |
	\let b:is_new = 1 | endif
" TODO why can't i just do this directly in above?
au BufWritePost *.sh if get(b:, 'is_new', 0) |
	\silent execute '!chmod +x %' | e | endif
au BufRead *.sh nmap <buffer> <F5> :w<cr>:!bash %<cr>
au BufRead *.sh imap <buffer> <F5> <Esc>:w<cr>!bash %<cr>

au BufNewFile *.sh 0r ~/.vim/skel.sh
au BufNewFile *.sh normal G
au BufNewFile *.sh startinsert

" TODO move cursor to inside setup and enter insert
au! BufNewFile,BufRead *.ino,*.pde setlocal ft=arduino
au Filetype arduino setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=2
" TODO augroup on (ft?) to only specify *.ino,*.pde once, and similarly for
" other languages?
" this had to come after the aus
au BufNewFile *.ino,*.pde 0r ~/.vim/skel.ino

au BufNewFile .envrc 0r ~/.vim/envrc_skel

" TODO is it actually appropriate to use au! here? i don't above...
" for tab delimiting keywords.txt, as supposed to
au! BufNewFile,BufRead keywords.txt setlocal ft=arduino_keywords_txt
au Filetype arduino_keywords_txt setlocal shiftwidth=8 noexpandtab softtabstop=0

" TODO fix how now ,H shortcut doesn't work to hide highlighting of end of long lines
" (now that i replaced commented lines below w/ these. do want the xml syntax
" highlighting tho)
au! BufNewFile,BufRead *.launch setlocal ft=xml
au Filetype xml setlocal expandtab tabstop=2 shiftwidth=2
"au! BufNewFile,BufRead *.launch setlocal ft=launch
"au Filetype launch setlocal expandtab tabstop=2 shiftwidth=2
""textwidth=0

set spellfile=$HOME/src/dotfiles/vim/spell/spellfile.utf-8.add
au! BufNewFile,BufRead *.md setlocal ft=markdown
au Filetype markdown setlocal expandtab tabstop=3 shiftwidth=3
" TODO TODO now that i'm disabling spell by default, make some hotkey
" to quickly toggle the spellcheck visuals (share w/ txt below)
au Filetype markdown setlocal nospell spelllang=en_us

" TODO maybe just use MarkdownPreview (when would i want to MarkdownPreviewStop?)
" Requires 'iamcco/markdown-preview.nvim' plugin
au Filetype markdown nmap <buffer> <F5> <Plug>MarkdownPreviewToggle

" To exclude spellcheck from certain .txt files
" TODO TODO TODO revert change that deleted the "Filetype special_txt..." line
au! BufNewFile,BufRead requirements.txt setlocal ft=special_txt

" Trying to include some txt settings for my usual habits of making lots of
" nested bulleted lists, with indents at one level, often with - as prefix.
" TODO make it more like markdown?
" TODO Filetype and FileType both valid?
au Filetype text setlocal expandtab tabstop=1 shiftwidth=1
" TODO TODO now that i'm disabling spell by default, make some hotkey
" to quickly toggle the spellcheck visuals
au Filetype text setlocal nospell spelllang=en_us

au Filetype yaml setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=0

" TODO set vs setlocal? BufFilePost in others too?
au BufNewFile,BufRead,BufFilePost *.cir setlocal filetype=spice

" This turns off automatic comment continuation on making new lines. Should
" cover this file (which will be linked to by ~/.vimrc), as well as any other
" vimscript.
autocmd FileType vim setlocal formatoptions-=c formatoptions-=r formatoptions-=o

au Filetype xml setlocal expandtab tabstop=2 shiftwidth=2 softtabstop=2

set backspace=indent,eol,start
set autoindent

" TODO TODO what was the purpose of this again?
if has("au")
  filetype plugin indent on
endif

" To turn off the preview window in YouCompleteMe (or other completions)
set completeopt-=preview

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Configuration for the ale plugin. Can use command :ALEInfo to troubleshoot.
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Need to install the following separately:
" flake8:
"  - `sudo apt install flake8` (works on 18.04)
" black:
" - `python3 -m pip install black` (works on 18.04)
"   - still seems to be accessible inside virtual environments when installed
"     this way, though only tested w/ the same python3...
"
"   - didn't want to use the snap installation option as i thought it might be
"     slower

" TODO want to address cursor-disappearing-near-warnings/errs issue, like:
" https://github.com/dense-analysis/ale/issues/1470
" before trying to enable linters again
" TODO also try pydocstyle, mypy, and maybe pylint (not sure about overlap w/
" flake8 on the last one)
" By default, it seems ['flake8', 'mypy', 'pylint', 'pyright'] linters would
" otherwise be enabled.
let g:ale_linters = {'python': []}
"let g:ale_linters = {'python': ['flake8']}
" See "How can I run linters only when I save files?" section in README
" https://github.com/dense-analysis/ale if I find the linters annoying while
" typing (the below is probably the option I want, but there were more).
"let g:ale_lint_on_text_changed = 'never'

" :ALEInfo doesn't seem to include supported fixers, but :ALEFixSuggest
" (found via ':h ale') has some, though not sure how complete.
" After manually invoking :ALEFix, :ALEInfo should include the corresponding
" command and exit code in the 'Command History:' section at the bottom. 127
" probably means the fixer was not found.
" TODO configure vim python line length settings to blacks default of 88 or
" change blacks to lower (leaning towards former...)
" TODO also try isort (+ compare w/ autoimport / reorder-python-imports)
" For flake8 and black to play nicer together, might need something like:
" https://black.readthedocs.io/en/stable/compatible_configs.html#flake8
let g:ale_fixers = {'python': ['black']}
" Because I prefer single quotes for strings by default.
let g:ale_python_black_options = '--skip-string-normalization'

" Black does seem to find pyproject.toml files in parent directories to the
" edited file.
let g:ale_fix_on_save = 0
" This :bar thing may not work on Windows
" https://vi.stackexchange.com/questions/3885
" TODO ideally, find some way to have the redraw happen right after ALEFix
" finishes, as it might take a variable amount of time depending on the fixers
" and the input...
" This sleeps for 200ms then forces redraw. Otherwise, takes a few seconds to
" update display unless you move the cursor or something.
nmap <F2> :ALEFix<cr> \| :sleep 200m<cr> \| :redraw!<cr>
"nmap <F2> :ALEFix<cr>

"let g:ale_warn_about_trailing_whitespace = 0

" TODO possible to configure black to ignore my "import ipdb; ipdb.set_trace()"
" lines? NO. would have to try post processing or something.

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

" For easier configuring on a file by file basis with the # vim: options line
" set modeline

" Trying the default spell suggestions out. May turn off.
" Too busy. Want a way to spellcheck quickly though, and then turn it off.
" set spell spelllang=en_us

" Might try to find better uses for F2/3 keys, etc. Plugin for these commands is
" no longer something I use.
"map <F2> :Eval<ENTER>
"map <F3> :Connect<ENTER>1<ENTER><ENTER>

" vim-markdown configuration
let g:vim_markdown_folding_disabled = 1

" TODO automatically delete through comment when deleting from middle of comment
" block? or just use gq?

" TODO shortcut/macro to insert date

" statusline default approximation below from:
" https://unix.stackexchange.com/questions/224771
" start of default statusline
set statusline=%f\ %h%w%m%r\ 
" NOTE: preceding line has a trailing space character

" these two lines were also in the SO answer, and pretty sure don't need first
"set statusline+=%#warningmsg#
" definitely don't need this line, but this might be a good addon
"set statusline+=%{SyntasticStatuslineFlag()}

" TODO TODO hack saving last edited file + function name (using same thing to get
" function name below) to some repo level or global file, so that pytest alias can run
" JUST this test, to skip potentially time intensive tests that i'm not working on (and
" also just to prevent other tests from confusing me when i'm reading the output)

" TODO TODO see also Taglist extension and others bookmarked around the same
" time. may ultimately provide a more reliable way to get function names.
" (or is there something jedi based? does jedi have a notion of scope?)
" TODO see also https://stackoverflow.com/questions/13634826
" and https://stackoverflow.com/questions/33699049

" TODO need to special case non python files here (to at least not append this
" part?) certain file types (ones where taghelper doesn't work)?
" (does taghelper actually *error* anywhere, or was it just that it wasn't installed?
" probably just wasn't installed...)

set statusline+=%*

" end of default statusline (with ruler)
set statusline+=%=%(%l,%c%V\ %=\ %P%)

" This will put the function name in square brackets at the far right in the statusline
" TODO maybe try to embed s.t. it's just to the right of the filename?
let &statusline .= ' %{taghelper#curtag()}'

" TODO just display func name in another color? (or flank w/ symbols like
" ex?)
"hi statusline ctermfg=5 ctermbg=0

" should make it so the statusline is always displayed
set laststatus=2

" should make mouse scrolling work inside tmux
" enables mouse (just scrolling? selection, etc?) in [a]ll modes
set mouse=a

nnoremap q :call SaveAndQuit()<CR>

" To list all <F*> mappings (https://vi.stackexchange.com/questions/20192):
function FKeyMaps()
    for i in range(1, 12)
        if !empty(mapcheck('<F'.i.'>'))
            execute 'map <F'.i.'>'
        endif
    endfor
endfunction

" Tried to have this as <leader><F1> but the mapping for <F1> is all that was triggered.
" Same with <F2>.
nmap <F10> :call FKeyMaps()<CR>

" To list all leader commands, use `:map <leader>` or `:verbose map <leader>`

" Leader commands added by plugins:
" - nerdcommenter: many with the prefix of <leader>c

" TODO TODO leader command for jumping to end of current indention block (i.e. the body
" of a long loop in python) (some plugin for this?)

" TODO TODO find a way (plugin?) to get/insert at current indentation level for commands
" that insert lines (python specific OK)

" TODO shortcut to select current line + next line, and then zg? (i do it a lot)

" TODO is this even necessary or does my 'o' + python filetype settings already handle
" as well as anything could?
function PyInsertAtCurrIndent()
    " TODO will this fn need modification to work both in the body of a fn and on the
    " line defining the fn? test.

    " https://stackoverflow.com/questions/14993012
    " Counted in spaces: https://stackoverflow.com/questions/46090914
    let l:curr_indent_spaces = indent(line('.'))

    " TODO TODO before attempting to improve behavior of just 'o'<what i want>, i need
    " to make sense of 'o' behavior. it seems there might be some (python) code-specific
    " thing at work already

    " TODO special handling for lines w/ whitespace only? necessary?

    " NOTE: the first branch of this is currently disabled b/c auto tab stuff doesn't
    " take effect for me since no text is entered after 'o' command before insert mode
    " is left (doesn't matter that there is `startinsert` at end)
    "if l:curr_indent_spaces != 0
    if 0 && l:curr_indent_spaces != 0
        echom "curr indent:" l:curr_indent_spaces "spaces"
        execute "normal! o\<esc>"
    else
        " TODO TODO TODO use something other than counting '.' in this tag, as it will
        " do nothing for any block contruct other than functions /
        " nests-of-only-functions!
        " TODO maybe search (upwards?) until we get a line w/ non-zero indent in the
        " same tag, and use that indent?
        echom "curr indent == 0"

        let l:curr_tag = taghelper#curtag()
        if len(l:curr_tag) == 0
            let l:curr_indent_multiple = 0
        else
            " https://vi.stackexchange.com/questions/21622
            let l:curr_indent_multiple = count(l:curr_tag, '.') + 1
        endif

        echom "curr indent multiple:" l:curr_indent_multiple

        " TODO is tabstop suppossed to be 4 for python? what's diff between it and
        " shiftwidth? any reason not to just use shiftwidth (what has seemed to matter
        " so far, as only it is 4 for me now in a python file, w/ tabstop=8).  also,
        " what's softtabstop / does it matter?
        let l:curr_indent_spaces = l:curr_indent_multiple * &l:shiftwidth

        echom "curr indent:" l:curr_indent_spaces "spaces"

        " https://vi.stackexchange.com/questions/9644
        " https://learnvimscriptthehardway.stevelosh.com/chapters/30.html
        execute "normal! o" . repeat(' ', l:curr_indent_spaces) . "\<esc>"
    endif

    " https://stackoverflow.com/questions/11587124
    startinsert

endfunction

" Custom functions on the 'Leader' keyboard
let mapleader = ","
" TODO are these spaces after <leader>[some char] functional (seems equiv to l
" interactively) or are the ignored here?
nnoremap <leader>b oimport ipdb; ipdb.set_trace()<Esc>k<CR>
"nnoremap <leader>b oimport ipdb; ipdb.set_trace()<Esc>kA<CR>
nnoremap <leader>B o# TODO delete<CR>import ipdb; ipdb.set_trace()<CR>#<Esc>kA<CR>
"nnoremap <leader>D o#TODO delete<CR>#<Esc>kA<CR>
nnoremap <leader>D o# TODO delete<Esc>k<CR>
"nnoremap <leader>b :call PyInsertAtCurrIndent()<CR>import ipdb; ipdb.set_trace()<Esc>

nnoremap <leader>p oprint(f'{=}')<Esc>3hi

" strip all trailing whitespace. https://vi.stackexchange.com/questions/454
" TODO also print a message saying what we did? ok as-is?
nnoremap <leader>s :let _s=@/<Bar>:%s/\s\+$//e<Bar>:let @/=_s<Bar><CR>

" TODO delete this one? never use it...
nnoremap <leader>S oimport sys; sys.exit()<Esc>
"nnoremap <leader>S i¯\_(ツ)_/¯<Esc>

" TODO maybe modify the # to some kind of autodetected comment character,
" dependent on filetype?
" TODO TODO TODO figure out how to use NERDCommenter for this. it should be possible.

nnoremap <leader>1 o# TODO 
nnoremap <leader>2 o# TODO TODO 
nnoremap <leader>3 o# TODO TODO TODO 
nnoremap <leader>4 o# TODO TODO TODO TODO 


" Using simpylfold plugin instead of this now.
" https://stackoverflow.com/questions/357785
"set foldmethod=indent
" To not fold inner stuff
"set foldnestmax=2

" TODO test this doesn't conflict w/ existing gutter stuff (the git plugin)
" + resolve/find a plugin for this if it does
" TODO TODO or could i condense the amount of horizontal space the two take up somehow?
" i just need a small fold indicator, and the git stuff could just color/draw on top of
" any fold indicator as appropriate?
" TODO TODO how to get this to not show when all folds are closed?
" TODO TODO likewise, make sure git gutter thing isn't taking up that column unless file
" is actually changed
set foldcolumn=1

" TODO TODO TODO how to maintain syntax highlighting and make fold text shorter?
" re: better folded line summaries, see:
" https://vi.stackexchange.com/questions/4627
" https://vi.stackexchange.com/questions/14481
" etc. maybe there is a plugin for this tho?
" (from googling "vim change how folds are displayed")
"
" re: syntax highlighting, googling seems to indicate it's not possible =(
" TODO TODO TODO maybe just fold everything below the first line then, and change my
" toggling hotkeys to move a line down first (and then back up after toggling)

" TODO shortcuts involving SimpylFoldDocstrings/SimpylFoldImports

" TODO have stuff shorter than some threshold (maybe that it fits on screen, but
" probably more like 2-3 screens worth) start unfolded? or everything start that way?

let g:SimpylFold_fold_docstring = 0
let g:SimpylFold_fold_blank = 1

" TODO TODO hotkeys for toggling all folds?
" https://vim.fandom.com/wiki/Folding

" NOTE: opening / closing one level of folds across whole file = zm / zr
" For toggling folds with space
nnoremap <space> za
vnoremap <space> zf

"nnoremap <leader><space> 

" TODO TODO is there really not a plugin / way to make foldlevel just the highest level
" currently present? if not, implement.
set foldlevelstart=3


let g:doge_doc_standard_python = 'google'
" TODO maybe disable doge default <leader>d mapping and make my own that first jumps to
" line w/ def/class for current fn/class before calling

" TODO make it some kind of function that takes input so flow can be:
" fn name <Enter> arg1, ..., argN <Enter>
" (with cursor then getting placed at appropriate indentation to start body)
" ...rather than haveing to change modes and navigate over to enter args
" (similar to how doge plugin uses tab to cycle through items to fill in)

" TODO maybe a separate version behind <leader>F to add docstring too?
nnoremap <leader>f o<cr>def ():<cr><Esc>k$2hi

" TODO TODO try to get this search (or the navigation commands below) to start from
" current cursor position (like w/ N/n in normal search)
"
" Populates quickfix list w/ all references (in current file?)
nnoremap <leader>g :YcmCompleter GoToReferences<CR>
" Should default to GoToDefinition if available.
nnoremap <leader>G :YcmCompleter GoTo<CR>
" These commands are to navigate through the results from the YcmCompleter search(es)
" above (at least for GoToReferences, maybe also GoTo in some cases?)
"
" For explanation of what <Plug> is: https://vi.stackexchange.com/questions/31012
"
" NOTE: see also qf_loc_[previous/next] if I end up using the "location list" in
" addition to the quickfix list
nmap <leader>N <Plug>(qf_qf_previous)
nmap <leader>n <Plug>(qf_qf_next)

" Replace all instances of word under cursor
" :YcmCompleter RefactorRename <name> would be preferable if it didn't time out...
" TODO modify so you only have to type the new name, not the '/g' at the end too
" TODO maybe look into replacing w/ 'subversive' plugin? it do what i think?
nmap <leader>r *:%s//

" TODO TODO did i have something like above for searching for whole word under cursor
" (i.e. as with :\<WORD\>)? where? add something like that?

" TODO maybe add <leader>R for RefactorRename if i get that to work OK

" TODO also make it so it positions this line at the top of the screen
" doesn't work
nnoremap <leader>m /def main(<Enter>zt

" TODO TODO TODO get one of the things below working for print inserts
" TODO maybe modify this to also select the current word, and copy that
" inside single quotes, with a colon, like my frequent printing convention?
" or a slightly different leader command for that?
" TODO be on the lookup for cases where wb -> vawy -> b doesn't bring you
" back to the same place wb did (or cases where wb doesn't get you at start of
" current word like i might want)
" (could read https://www.reddit.com/r/vim/comments/1xzfjy more thoroughly, re:
" wb edge cases)
" The wb part is to go to the beginning of the current word
" TODO TODO why does x not seem to be behaving here? (seems to paste last thing
" i yanked, if any)
"nnoremap <leader>p vawxiprint(<Esc>pi)<Esc>
"nnoremap <leader>p wbiprint()<Esc>hi
"nnoremap <leader>P wbivawybiprint('<Esc>pi:')<Esc>hi
" TODO cmds for inserting "if <ins>:" and "return <ins>" and maybe try: ...
" except <ins>: (or ins in try block for that one? leaning towards latter)

" TODO maybe also load the strings these enter into yank before,
" for pasting closer like i often do when entering opener manually?
" TODO maybe use alt or some other modifier (if avail) to switch between o and O
" insert?
nnoremap <leader>' o'''<Esc>
nnoremap <leader>" o"""<Esc>
nnoremap <leader>` o```<Esc>
" TODO maybe 87/89? maybe get from that vim variable (textwidth)?
" (88 might have been making last char wrap)
" maybe also add more blank lines (on both sides?)?
nnoremap <leader># o<Esc>87i#<Esc>
" TODO TODO some leader cmds for inserting docstring lines (settling on a
" convention i want to use first, from the ~2 big ones)

nnoremap <leader>h :call ToggleH()<CR>

" TODO delete if this doesn't end up feeling smoother / if i find a single key solution
" (which i'd prefer) i'm happy with
" update = write if modified
" TODO if i like, make also add <leader>q and remove my current hacky setup preventing
" use of q for macros
nnoremap <leader>w :update<CR>

" This function is provided by the vim-better-whitespace plugin
nnoremap <leader>W :ToggleWhitespace<CR>

" TODO if i actually do get a shortcut i'm happy with for invoking my python scripts,
" maybe also add one for profiling them with kernprof?
" 'l' for [l]ine_profiler (invoked from CLI as `kernprof -l -v <x>.py` after installing)
" This decorator is required to mark functions to profile.
" TODO maybe save on this one too?
nnoremap <leader>l O@profile<Esc>

" TODO shortcut to link to visually selected lines w/ snippet highlight function in
" github. ideally in a way s.t. if online is out of date but that snippet hasn't
" changed, it uses the git information to translate to old line numbers somehow?

" Technically this will toggle all of them, but I'm assuming that (in the current file)
" ALL are EITHER commented OR uncommented when run, otherwise it won't guarantee they
" are all commented to be run without kernprof injecting 'profile' into builtins and
" might behave in some other weird ways.
function ToggleProfileDecCommentState()
    " The trailing 'e' prevents one search term not being found from causing whole
    " function to fail.
    " https://vi.stackexchange.com/questions/10821
    %s/#@profile/LPROF_DEC_PLACEHOLDER/ge
    %s/@profile/#@profile/ge
    %s/LPROF_DEC_PLACEHOLDER/@profile/ge
endfunction

nnoremap <leader>k :call ToggleProfileDecCommentState()<CR>

" Since I always forget the syntax for redirecting to std[e]rr
nnoremap <leader>e o>&2<Esc>


" TODO maybe add a hotkey for "sourcing" vimrc if not already one

" TODO TODO if i dont improve the zg behavior w/ the various syntaxes for python
" multiline strings, maybe make leader cmd to fix behavior for some of the more
" common syntaxes that currently have problems

" TODO maybe add <leader> commands for inserting common (groups of?) import
" statements (:source ~/.vimrc  OR  :source $MYVIMRC )
" (but might need to modify the above so all commands are replaced, rather than
" triggering warnings in vim)
" TODO maybe simplest way would just be to try to get vim to save and reopen at
" current position?

" TODO work? trying to get easy copy paste.
" adapted from: https://vi.stackexchange.com/questions/36781
" see also: https://vim.fandom.com/wiki/Accessing_the_system_clipboard
"
" mabye really isn't gonna work w/ just Ctrl prefix? despite one guy in comments somehow
" having it work for him? wasn't working for me when testing w/ vim inside tmux.
" does seem like it could be a tmux display/terminal issue. bit unclear.
" see:
" https://gist.github.com/mikeboiko/b6e50210b4fb351b036f1103ea3c18a9
" https://unix.stackexchange.com/questions/591293/how-do-i-correctly-reset-display
" https://goosebearingbashshell.github.io/2017/12/07/reset-display-variable-in-tmux.html
" https://superuser.com/questions/1742713/how-to-pipe-vim-clipboard-through-tmux
" yea, it works in a fresh tmux session, where DISPLAY is still :0
" i must have [connected (this alone change it?) / started] via ssh the other tmux
" session, which seems to have changed DISPLAY to something else.
" so far i could only get DISPLAY to be non-:0 if starting tmux from ssh
"nnoremap <C-c> "+y  " Normal (must follow with an operator)
"xnoremap <C-c> "+y  " Visual
" M=meta (alt key). S=shift?
" not working? i have clipboard and x11 support...
nnoremap <M-S-c> "+y  " Normal (must follow with an operator)
xnoremap <M-S-c> "+y  " Visual
" TODO do for Ctrl[-Shift]-v too, thought that seems to maybe already work? at least in
" some modes

" <leader>c might be used by some nerd commenter stuff, but not sure i actually want to
" try using that extension...
noremap <leader>y "+y
noremap <leader>v "+p
" TODO delete these ones? useful? * is x11 selection buffer, right?
noremap <leader>Y "*y
noremap <leader>V "*p


noremap <F12> <Esc>:syntax sync fromstart<CR>
inoremap <F12> <C-o>:syntax sync fromstart<CR>

" TODO are my .sh tab settings correct?
" (apparently, some spec says to use tabs?)

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
function XTermPasteBegin()
  set pastetoggle=<Esc>[201~
  set paste
  return ""
endfunction

" TODO any reason not to always have this set?
" This is to more easily read through files with really long lines, like soem
" python logging outputs. Otherwise, lines beneath current that would be wrapped
" are not displayed at all, only displaying the @ character at the start...
set display+=lastline

" TODO fix default state of this when opening files where there are long lines
" but there is not the 't' formatoption (currently shows red for me in a cpp
" file, but i can't switch off w/o enabling that formatoption b/c my check)
" At end to ensure it happens after any filetypes are redefined at load.
call ToggleH()

" TODO fix so there is never any of the highlighting associated with this for CSV files
"au Filetype csv call ToggleH()

