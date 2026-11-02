#!/usr/bin/env python3
"""Syncs the untracked ~/.bash.local with the tracked bash.local.template.

Active statements in ~/.bash.local that the template lacks are appended to the template,
commented out. Nothing is committed, so review those with `git diff`.

Statements in the template that ~/.bash.local lacks are warned about, and a prompt asks
whether to append them to ~/.bash.local, commented out.

Variables, aliases and functions are matched by name, so differing values don't count as
differences. Anything else (e.g. `if` blocks, `source` lines, or edits like
PATH="/foo:$PATH", where one name can have many lines) is matched by its text.

A commented-out statement counts as present in either file, so a machine can note it
doesn't use something without being warned about it every run. One that is only
commented out in ~/.bash.local is not merged into the template.

Commented-out code is told apart from prose by having no space after the '#'.
Variables, aliases and functions are also recognized with a space there.
"""

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
import sys
import textwrap

TEMPLATE = Path(__file__).resolve().parent / 'bash.local.template'
LOCAL = Path.home() / '.bash.local'

DECLARATIONS = ('export', 'declare', 'typeset', 'readonly')

COMMENT_RE = re.compile(r'(\s*#+( ?))(.*)')
PROSE_RE = re.compile(r'(TODO|NOTE|FIXME|XXX)\b')
ASSIGNMENT_RE = re.compile(r'([A-Za-z_]\w*)(\+?=)(.*)', re.S)
FUNCTION_RE = re.compile(r'(?:function\s+)?([\w:.+-]+)\s*\(\s*\)|function\s+([\w:.+-]+)')

# Since the template is tracked, values of variables named like this aren't copied to it.
SECRET_RE = re.compile(
    r'''(?<![^\s#])(\w*(?:TOKEN|SECRET|PASSWORD|PASSWD|API_?KEY|ACCESS_KEY|PRIVATE_KEY'''
    r'''|CREDENTIAL)\w*=)(?:"[^"]*"|'[^']*'|[^\s"'])*''',
    re.I
)


@dataclass
class Entry:
    """A top-level statement in one of the files.

    Attributes:
        keys: (kind, name) pairs identifying what the statement defines.
        lines: the statement's lines, as they are in the file.
        leading: prose comment lines directly above the statement.
        commented: whether the statement is commented out.
    """
    keys: list
    lines: list
    leading: list
    commented: bool


def is_complete(code):
    """Returns whether code has no unterminated statements, according to bash.
    """
    if re.search(r'(?<!\\)(?:\\\\)*\\$', code):
        return False

    # stderr is for unterminated heredocs, which only cause a warning.
    # extglob because interactive shells typically have it on, and it changes the syntax.
    result = subprocess.run(['bash', '-O', 'extglob', '-n'], input=f'{code}\n',
        capture_output=True, text=True
    )
    return result.returncode == 0 and not result.stderr


def split_words(code):
    """Returns the words of code (quotes included), or None if not one simple command.
    """
    code = code.replace('\\\n', ' ')
    words = []
    word = ''
    # What would close each of the quotes / expansions we are currently inside of
    closers = []
    i = 0
    while i < len(code):
        char = code[i]
        pair = code[i:i + 2]
        closer = closers[-1] if closers else None
        step = 1

        if closer == "'":
            if char == "'":
                closers.pop()

        elif char == '\\':
            step = 2

        elif pair in ('$(', '${'):
            closers.append(')' if pair == '$(' else '}')
            step = 2

        elif char == closer:
            closers.pop()

        elif char in '"\'`' and closer != '"':
            closers.append(char)

        elif closer is None:
            if char.isspace():
                if word:
                    words.append(word)
                    word = ''
                i += 1
                continue

            if char == '#' and not word:
                break

            if char == '(' and word.endswith('='):
                closers.append(')')

            elif char == ';' and re.fullmatch(r'\s*(#.*)?', code[i + 1:], re.S):
                break

            elif char in ';&|<>()':
                return None

        word += code[i:i + step]
        i += step

    if closers:
        return None

    return words + [word] if word else words


def statement_keys(code):
    """Returns (kind, name) pairs identifying what the statement in code defines.

    The kind is one of 'variable', 'alias', 'function' or 'statement'. For the last, the
    name is the text of the statement with whitespace normalized.
    """
    code = code.strip()
    other = [('statement', ' '.join(code.split()))]

    function = FUNCTION_RE.match(code)
    if function:
        return [('function', function.group(1) or function.group(2))]

    words = split_words(code)
    if not words:
        return other

    if words[0] == 'alias':
        aliases = [re.fullmatch(r'([^\s=]+)=.*', word, re.S) for word in words[1:]]
        if aliases and all(aliases):
            return [('alias', match.group(1)) for match in aliases]
        return other

    args = words
    if words[0] in DECLARATIONS:
        args = [word for word in words[1:] if not word.startswith('-')]

    assignments = [ASSIGNMENT_RE.fullmatch(word) for word in args]
    if not assignments or not all(assignments):
        return other

    for match in assignments:
        name, operator, value = match.groups()
        if operator == '+=' or re.search(rf'\${{?{name}\b', value):
            return other

    return [('variable', match.group(1)) for match in assignments]


def read_active(lines, start, path):
    """Returns (number of lines, code) of the statement starting at lines[start].
    """
    for end in range(start + 1, len(lines) + 1):
        code = '\n'.join(lines[start:end])
        if is_complete(code):
            return end - start, code

    sys.exit(f'{path}:{start + 1}: syntax error or unterminated statement. Changed nothing.')


def read_commented(lines, start):
    """Returns (number of lines, code) of commented-out code starting at lines[start].

    Returns None if lines[start] is prose.
    """
    prefix, space, first = COMMENT_RE.fullmatch(lines[start]).groups()
    if not first.strip() or PROSE_RE.match(first):
        return None

    if space and statement_keys(first)[0][0] == 'statement':
        return None

    code = []
    for line in lines[start:]:
        if not line.startswith(prefix):
            break

        code.append(line[len(prefix):])
        if is_complete('\n'.join(code)):
            return len(code), '\n'.join(code)

    # Unterminated, so probably prose with an apostrophe or similar
    return None


def parse(path):
    """Returns list of Entry for each statement in path, commented-out or not.
    """
    lines = path.read_text().splitlines()
    entries = []
    prose = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if not line.strip() or (i == 0 and line.startswith('#!')):
            prose = []
            i += 1
            continue

        commented = line.lstrip().startswith('#')
        statement = read_commented(lines, i) if commented else read_active(lines, i, path)
        if statement is None:
            prose.append(line)
            i += 1
            continue

        n_lines, code = statement
        entries.append(
            Entry(statement_keys(code), lines[i:i + n_lines], prose, commented)
        )
        prose = []
        i += n_lines

    return entries


def commented_block(entry):
    """Returns text for entry and its leading comments, with the statement commented out.
    """
    lines = entry.lines
    if not entry.commented:
        lines = [f'#{x}' for x in textwrap.dedent('\n'.join(lines)).split('\n')]

    return SECRET_RE.sub(r'\1"<redacted>"', '\n'.join(entry.leading + lines))


def append_entries(path, entries):
    """Appends commented-out entries to path, each after a blank line.
    """
    existing = path.read_text().rstrip('\n')
    blocks = [commented_block(entry) for entry in entries]
    path.write_text('\n\n'.join([existing, *blocks]).lstrip('\n') + '\n')


def print_keys(entries, exclude):
    """Prints description of each key in entries that is not in exclude, once each.
    """
    keys = dict.fromkeys(k for entry in entries for k in entry.keys if k not in exclude)
    for kind, name in keys:
        print(f'  {kind} {textwrap.shorten(name, width=70, placeholder="...")}')


def main():
    if not LOCAL.exists():
        sys.exit(f'{LOCAL} does not exist. ./link_only creates it from the template.')

    local = parse(LOCAL)
    template = parse(TEMPLATE)
    local_keys = {key for entry in local for key in entry.keys}
    template_keys = {key for entry in template for key in entry.keys}

    to_template = [x for x in local
        if not x.commented and not template_keys.issuperset(x.keys)
    ]
    to_local = [x for x in template if not local_keys.issuperset(x.keys)]

    if to_template:
        append_entries(TEMPLATE, to_template)
        print(f'Added to {TEMPLATE.name}, commented out:')
        print_keys(to_template, template_keys)
        print('Values are copied as they are, unless the variable is named like a secret.'
            f' Check `git diff {TEMPLATE.name}` before committing.'
        )

    if to_local:
        print(f'Warning: in {TEMPLATE.name} but not {LOCAL}:')
        print_keys(to_local, local_keys)
        try:
            answer = input(f'Add these to {LOCAL}, commented out? [y/N] ')
        except EOFError:
            answer = ''
            print()

        if answer.strip().lower() in ('y', 'yes'):
            append_entries(LOCAL, to_local)
            print(f'Added to {LOCAL}.')

    if not to_template and not to_local:
        print(f'{TEMPLATE.name} and {LOCAL} are in sync.')


if __name__ == '__main__':
    main()
