
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

" TODO TODO TODO why was this not changing which python is actually used?
"let g:ycm_server_python_interpreter = '/usr/bin/python3.6'

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
"Plugin 'Valloric/YouCompleteMe'

" Trying to see if feature added in
" https://github.com/VundleVim/Vundle.vim/pull/604 supports installing an older
" version of YCM (referenced by commit, for now), as I currently need to get it
" to work with VIM <8.2 (7.4 [maybe patched to ~8?] vim-gtk).
" TODO might need to update vundle for this to work?
" TODO could also just try forking and keeping my HEAD at this commit if i can't
" get this to work
" at this this syntax didn't work
"Plugin 'Valloric/YouCompleteMed@d98f896'

" My fork at d98f896.
" TODO TODO TODO still getting the same error as in ppa case!!! fix!
" YouCompleteMe unavailable: invalid syntax (vimsupport.py, line 184)
" TODO maybe i just need to build YouCompleteMe myself again, or at lesat check
" that i still have any requried dependencies (before installing maybe?)
" TODO might try finding last commit still w/ 3.5 support and then just using
" sysem python3. i tried using a 3.6 venv but that didn't work
" python3 install.py. also tried `python3.6 install.py --clang-completer`.
" https://github.com/ycm-core/YouCompleteMe/issues/3711 seems to indicate it
" should work, but maybe that support was added after the commit i reverted to?
" ok so https://github.com/ycm-core/YouCompleteMe/issues/3732
" `:py3 print( __import__( 'sys' ).version )` -> 3.5, despite 3.6 being used for
" build. why?
" I also tried setting the path to the inteprete to match build python (see
" above), but it didn't seem to change anything:
" https://github.com/ycm-core/YouCompleteMe/issues/2917
" https://github.com/ycm-core/YouCompleteMe/issues/2136
Plugin 'tom-f-oconnell/YouCompleteMe'

Plugin 'dense-analysis/ale'

" For "aligning text" figure out how to use.
Plugin 'godlygeek/tabular'

" Shows indicator in left col of which lines differ from HEAD, etc.
" TODO delete if i don't like it. haven't used much.
Plugin 'airblade/vim-gitgutter'

" TODO what does this do again?
Plugin 'henrik/vim-indexed-search'

" Provides the cfi (current function info) function used below, in custom
" statusline.
" TODO (as per comments below) consider trying to replace w/ tagstack,etc
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

" https://stackoverflow.com/questions/57014805
function! IsWSL()
  if has("unix")
    let lines = readfile("/proc/version")
    if lines[0] =~ "Microsoft"
      return 1
    endif
  endif
  return 0
endfunction

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
function! QuitIfNotModifiedOrStartRecording()
    " https://stackoverflow.com/questions/13107453
    if ! &mod
        quit
    else
        " https://vi.stackexchange.com/questions/7844
        " https://stackoverflow.com/questions/43654089
        let c = nr2char(getchar())
        " (i don't think i actually want to leave the macro recording option
        " here, as usually it just ends up being frustrating when i'm trying to
        " quit. may want to rebind either the quit shortcut / macro recording,
        " if i end up wanting the option to record macros.)
        "execute 'normal! q'.c
    endif
endfunction

" To settle the confusion I've had as to how to tell whether lines will
" hard-wrap (with appropriate EOL characters inserted).
" Also seems that testing this will require testing more than just one thing,
" hence the function.
" Some references on hard wrapping behavior:
" https://stackoverflow.com/questions/1290285
" http://vimdoc.sourceforge.net/htmldoc/change.html#fo-table
" https://vim.fandom.com/wiki/Automatic_word_wrapping
function! WillBreakLines()
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
function! ToggleH()
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
filetype plugin indent on

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
" but opening ft=launch, i get 'tcq'.
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
au BufRead *.py nmap <F5> :w<cr>:!./%<cr>
au BufRead *.py imap <F5> <Esc>:w<cr>!./%<cr>
"au BufRead *.py nmap <F5> :w<cr>:!python %<cr>
"au BufRead *.py imap <F5> <Esc>:w<cr>!python %<cr>

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
au BufRead *.sh nmap <F5> :w<cr>:!bash %<cr>
au BufRead *.sh imap <F5> <Esc>:w<cr>!bash %<cr>

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

au! BufNewFile,BufRead *.launch setlocal ft=launch
au Filetype launch setlocal expandtab tabstop=2 shiftwidth=2
"textwidth=0

set spellfile=$HOME/src/dotfiles/vim/spell/spellfile.utf-8.add
au! BufNewFile,BufRead *.md setlocal ft=markdown
au Filetype markdown setlocal expandtab tabstop=3 shiftwidth=3
" TODO TODO now that i'm disabling spell by default, make some hotkey
" to quickly toggle the spellcheck visuals (share w/ txt below)
au Filetype markdown setlocal nospell spelllang=en_us

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

" TODO TODO TODO hack saving last edited file + function name (using same thing 
" to get function name below) to some repo level or global file, so that pytest
" alias can run JUST this test, to skip potentially time intensive tests that
" i'm not working on (and also just to prevent other tests from confusing me
" when i'm reading the output)
"
" TODO shortcut to toggle old statusline?

" TODO TODO see also Taglist extension and others bookmarked around the same
" time. may ultimately provide a more reliable way to get function names.
" (or is there something jedi based? does jedi have a notion of scope?)
" TODO see also https://stackoverflow.com/questions/13634826
" and https://stackoverflow.com/questions/33699049

" TODO need to special case non python files here (to at least not append this
" part?) certain file types (ones where cfi doesn't work)?
" (does cfi actually *error* anywhere, or was it just that it wasn't installed?
" probably just wasn't installed...)

" (NOTE: these fixes are more important how that i'm trying to use this to hack
" together something that runs only the most recent test i was editing in
" pytest)
" TODO TODO possible to work in whitespace / empty lines in function too?
" (doesn't now)
" TODO TODO fix false positives (e.g. in choice_analysis, in top-level code
" after press, the status bar says we are still in the function press)
" Show the current function name on statusline. Works* for at least Python.
" TODO only enable this custom statusline (any of the mods, not just the
" function name line) if python?
" `cfi` requires the addon https://github.com/tyru/current-func-info.vim
"set statusline+=' %{cfi#format("%s", "")}'

set statusline+=%*

" end of default statusline (with ruler)
set statusline+=%=%(%l,%c%V\ %=\ %P%)

" TODO just display func name in another color? (or flank w/ symbols like
" ex?)
"hi statusline ctermfg=5 ctermbg=0

" should make it so the statusline is always displayed
set laststatus=2

" should make mouse scrolling work inside tmux
" enables mouse (just scrolling? selection, etc?) in [a]ll modes
set mouse=a

nnoremap q :call QuitIfNotModifiedOrStartRecording()<CR>

" TODO shortcut to select current line + next line, and then zg? (i do it a lot)

" Custom functions on the 'Leader' keyboard
let mapleader = ","
" TODO are these spaces after <leader>[some char] functional (seems equiv to l
" interactively) or are the ignored here?
nnoremap <leader>b oimport ipdb; ipdb.set_trace()<Esc>
nnoremap <leader>s oimport sys; sys.exit()<Esc>
" TODO check i'm not shadowing any possibly-useful pre-existing commands after
" leader (or is leader entirely for custom commands?)
" TODO maybe modify the # to some kind of autodetected comment character,
" dependent on filetype?
nnoremap <leader>t o# TODO 
nnoremap <leader>m o# TODO maybe 
nnoremap <leader>c o#<Esc>
nnoremap <leader>d o"""<cr>"""<Esc>kA
nnoremap <leader>f o<cr>def ():<cr><Esc>k$2hi

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
function! XTermPasteBegin()
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

