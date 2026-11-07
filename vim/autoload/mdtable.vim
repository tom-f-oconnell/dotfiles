vim9script
# Aligns markdown (GFM pipe) table columns for display, by inserting padding as virtual
# text (text properties), without changing the buffer. Cell widths are measured as
# displayed with conceallevel=2 (hidden markup takes no space), so call mdtable#Render()
# only when that is in effect, and mdtable#Clear() when showing raw text (see vimrc).

if empty(prop_type_get('mdtable_pad'))
  prop_type_add('mdtable_pad', {})
endif

const delim_re = '^\s*|\=\s*:\=-\+:\=\s*\%(|\s*:\=-\+:\=\s*\)*|\=\s*$'
# Lines that could be part of a table, or start/end a fenced code block.
const candidate_re = '|\|^\s*\%(```\|\~\~\~\)'
const fence_re = '^\s*\zs\%(`\{3,}\|\~\{3,}\)'

export def Clear()
  prop_remove({type: 'mdtable_pad', all: true})
enddef

export def Render()
  var pads: list<list<any>> = []
  var lines = getline(1, '$')
  var n = len(lines)
  # Fenced code blocks are tracked here (like vim-markdown's syntax does), because
  # checking the syntax at each table can be slow (it may reparse from far above).
  var fence_end = ''
  var i = match(lines, candidate_re)
  while i >= 0
    var line = lines[i]
    if !empty(fence_end)
      if line =~# fence_end
        fence_end = ''
      endif
    elseif line =~# fence_re
      var fence = matchstr(line, fence_re)
      fence_end = '^\s*' .. escape(fence, '~') .. '[' .. fence[0] .. ']*\s*$'
    # Delimiter row at i, header row at i - 1.
    elseif i > 0 && stridx(line, '|') >= 0 && line =~# delim_re
        && stridx(lines[i - 1], '|') >= 0
      var last = i + 1
      while last < n && stridx(lines[last], '|') >= 0
        last += 1
      endwhile
      pads->extend(AlignTable(lines, i - 1, i, last - 1))
      i = last - 1
    endif
    i = match(lines, candidate_re, i + 1)
  endwhile
  Clear()
  for [lnum, col, text] in pads
    prop_add(lnum, col, {type: 'mdtable_pad', text: text})
  endfor
enddef

# Each cell as [start, end, closed]: 0-based byte range between pipes, and whether a
# pipe follows it. Text before a leading pipe, or after a trailing one, is not a cell.
def Cells(line: string): list<list<number>>
  var pipes: list<number> = []
  var pos = match(line, '\\\@<!|')
  while pos >= 0
    pipes->add(pos)
    pos = match(line, '\\\@<!|', pos + 1)
  endwhile
  if empty(pipes)
    return []
  endif
  var cells: list<list<number>> = []
  var leading = strpart(line, 0, pipes[0] + 1) =~# '^\s*|$'
  var start = leading ? pipes[0] + 1 : 0
  for p in pipes[(leading ? 1 : 0) :]
    cells->add([start, p, 1])
    start = p + 1
  endfor
  if strpart(line, start) !~# '^\s*$'
    cells->add([start, len(line), 0])
  endif
  return cells
enddef

# Display width of bytes [start, end) of line, with conceallevel=2. Rather than asking the syntax
# what is concealed (synconcealed() per character is too slow for big tables), this
# applies simplified versions of vim-markdown's concealing patterns.
def Width(line: string, start: number, end: number): number
  var text = strpart(line, start, end - start)
  if text !~# '[`*_[<~]'
    return strwidth(text)
  endif
  # A cell after a pipe isn't at the start of the line, which emphasis checks for.
  var prefix = start > 0 ? '|' : ''
  var t = prefix .. text
  # Code (incl. HTML code/pre and strikethrough, which vim-markdown highlights as code)
  # first, replacing contents with placeholders so they aren't treated as markup.
  t = substitute(t, '\\\@<!``\(.\{-}\)``', (m) => Blank(m[1]), 'g')
  t = substitute(t, '\\\@<!`\([^`]*\)`', (m) => Blank(m[1]), 'g')
  t = substitute(t, '<\(code\|pre\)\%(\s[^>]*\)\=>\(.\{-}\)</\1>', (m) => Blank(m[2]), 'g')
  t = substitute(t, '\\\@<!\~\~\(.\{-}\)\~\~', (m) => Blank(m[1]), 'g')
  # Links and images: only their text is shown.
  t = substitute(t, '\\\@<!!\=\[\([^]]*\)\]\s*\%(([^)]*)\|\[[^]]*\]\)', '\1', 'g')
  # Emphasis markers.
  t = substitute(t, '\%(^\|\s\)\zs\(\*\*\*\|\*\*\|\*\|___\|__\|_\)\(\S\%(.\{-}\S\)\=\)\1'
    .. '\ze\%(\W\|$\)', '\2', 'g')
  return strwidth(t) - len(prefix)
enddef

def Blank(text: string): string
  return repeat('x', strwidth(text))
enddef

# Arguments are 0-based indices into lines. Returns padding to add, as a list of
# [lnum, col, text] for prop_add().
def AlignTable(
    lines: list<string>, header: number, delim: number, last: number): list<list<any>>
  var pads: list<list<any>> = []
  var rows: list<list<list<number>>> = []
  var widths: list<number> = []
  for r in range(header, last)
    var cells = Cells(lines[r])
    for j in range(len(cells))
      var w = Width(lines[r], cells[j][0], cells[j][1])
      cells[j]->add(w)
      if j >= len(widths)
        widths->add(w)
      elseif w > widths[j]
        widths[j] = w
      endif
    endfor
    rows->add(cells)
  endfor

  var aligns: list<string> = []
  for cell in rows[delim - header]
    var spec = trim(strpart(lines[delim], cell[0], cell[1] - cell[0]))
    aligns->add(spec =~# '^:.*:$' ? 'c' : spec =~# ':$' ? 'r' : 'l')
  endfor

  for r in range(header, last)
    var line = lines[r]
    var cells = rows[r - header]
    for j in range(len(cells))
      var [cs, ce, closed, w] = cells[j]
      var pad = widths[j] - w
      if pad <= 0 || !closed
        continue
      endif
      if r == delim
        # Extend the dashes, keeping any alignment colon and spaces after them.
        pads->add([r + 1, strridx(strpart(line, 0, ce), '-') + 2, repeat('-', pad)])
        continue
      endif
      var align = get(aligns, j, 'l')
      var before = align ==# 'r' ? pad : align ==# 'c' ? pad / 2 : 0
      var content = match(strpart(line, 0, ce), '\S', cs)
      if before > 0 && content >= 0
        pads->add([r + 1, content + 1, repeat(' ', before)])
      else
        before = 0
      endif
      if pad > before
        pads->add([r + 1, ce + 1, repeat(' ', pad - before)])
      endif
    endfor
  endfor
  return pads
enddef
