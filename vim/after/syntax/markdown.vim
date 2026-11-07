" Extra concealing for vim-markdown, shown when conceallevel > 0 (see vimrc), to render
" list bullets and task-list checkboxes.
syn match mkdBullet /^\s*\zs[-*+]\ze\s/ contained containedin=mkdListItem conceal cchar=•
syn match mkdCheckbox /\%(^\s*\%([-*+]\|\d\+\.\)\s\+\)\@<=\[ \]\ze\s/ contained
  \ containedin=mkdListItemLine conceal cchar=☐
syn match mkdCheckboxDone /\%(^\s*\%([-*+]\|\d\+\.\)\s\+\)\@<=\[[xX]\]\ze\s/ contained
  \ containedin=mkdListItemLine conceal cchar=☑
