" vim:foldmethod=marker:fen:
scriptencoding utf-8


if get(g:, 'cfi_disable') || get(g:, 'loaded_cfi_ftplugin_zsh')
    finish
endif
let g:loaded_cfi_ftplugin_zsh = 1

" Saving 'cpoptions' {{{
let s:save_cpo = &cpo
set cpo&vim
" }}}

let s:BEGIN_PATTERN = '\C'.'^\s*'.'\%(function\s\+\)\?'.'\(\S\+\)'.'\s*()'
let s:finder = cfi#create_finder('zsh')

function! s:finder.find(ctx) "{{{
    let NONE = ''

    if search(s:BEGIN_PATTERN, 'bW') == 0
        return NONE
    endif

    let m = matchlist(getline('.'), s:BEGIN_PATTERN)
    if empty(m)
        return NONE
    endif

    return m[1]
endfunction "}}}

call cfi#register_simple_finder('zsh', s:finder)
unlet s:finder




let &cpo = s:save_cpo
