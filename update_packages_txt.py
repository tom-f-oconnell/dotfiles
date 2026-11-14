#!/usr/bin/env python3
"""Proposes updates to packages.txt from the apt packages installed by hand here.

Shows the change as a diff, and only writes packages.txt after a yes at the prompt.

packages.txt has one package per line. A line `#name` is a package that is known but
left out (it is not installed on a new machine, and is not proposed again). To prune
a package for good, comment it out rather than deleting its line. Any other comment
or blank line is kept as is.

"Installed by hand" means marked manual in apt (`apt-mark showmanual`) and not part of
the system as first installed (/var/log/installer/initial-status.gz). Release upgrades
also mark some packages manual, so those can show up here too.
"""

import argparse
import datetime
import difflib
import gzip
import re
import subprocess
import sys
from pathlib import Path

PACKAGES_TXT = Path(__file__).resolve().parent / 'packages.txt'
INITIAL_STATUS = Path('/var/log/installer/initial-status.gz')
UBUNTU_HOSTS = ('ubuntu.com', 'canonical.com')
PACKAGE_LINE = re.compile(r'^(#?)\s*([a-z0-9][a-z0-9+.:-]*)\s*$')

HEADER = """\
# apt packages to install on a new machine: one per line.
# `#name` is a package known but left out. To prune one for good, comment it out
# rather than deleting its line, or ./update_packages_txt.py will propose it again.
"""

SECTIONS = (
    ('main', 'Installed by hand'),
    ('library', 'Libraries and headers installed by hand (lib*, *-dev)'),
    ('needed',
     'Left out: a package that is not yours depends on these, so they likely came with\n'
     '# the system or a release upgrade (uncomment any you do want)'),
    ('third_party',
     'Left out: from a repository outside Ubuntu, which has to be added before these\n'
     '# can be installed'),
    ('unavailable',
     'Left out: installed here, but no repository this machine knows has them anymore'),
)


def run(*args):
    return subprocess.run(args, capture_output=True, text=True, check=True).stdout


def installed_by_hand():
    """Returns the set of manually installed packages not in the initial install."""
    manual = set(run('apt-mark', 'showmanual').split())
    if not INITIAL_STATUS.exists():
        sys.exit(f'{INITIAL_STATUS} is missing, so the packages the system came with '
                 'cannot be told from yours.')

    with gzip.open(INITIAL_STATUS, 'rt', errors='replace') as f:
        initial = {line.split()[1] for line in f if line.startswith('Package: ')}

    return manual - initial


def origins(packages):
    """Returns {package: 'ubuntu' | 'third_party' | 'unavailable'} from apt's sources."""
    out = {}
    blocks = re.split(r'^(?=\S)', run('apt-cache', 'policy', *sorted(packages)),
                      flags=re.M)
    for block in blocks:
        if not block.strip():
            continue
        name = block.split(':', 1)[0].strip()
        urls = re.findall(r'^\s+-?\d+ (https?://\S+)', block, flags=re.M)
        if not urls:
            out[name] = 'unavailable'
        elif any(host in url for url in urls for host in UBUNTU_HOSTS):
            out[name] = 'ubuntu'
        else:
            out[name] = 'third_party'
    return out


def reverse_depends(package, *options):
    """Returns the installed packages that need `package` itself, not an alternative."""
    lines = run('apt-cache', 'rdepends', '--installed', '--no-suggests',
                '--no-conflicts', '--no-breaks', '--no-replaces', '--no-enhances',
                *options, package).splitlines()[2:]
    # `|name` needs this package or another, and `name:arch` is another build of it
    return {line.strip() for line in lines} - {package} - {
        name for name in map(str.strip, lines) if name.startswith('|') or ':' in name}


def needed_by_others(package, by_hand):
    """Returns whether the system would have `package` installed without being asked.

    That is, an installed package outside `by_hand` depends on it, or one of the
    ubuntu-* metapackages recommends it.
    """
    if reverse_depends(package, '--no-recommends') - by_hand:
        return True

    return any(name.startswith('ubuntu-') for name in reverse_depends(package))


def classify(packages, by_hand):
    """Returns {section key: sorted packages} for the packages to add."""
    origin = origins(packages) if packages else {}
    sections = {key: [] for key, _ in SECTIONS}
    for package in sorted(packages):
        if origin.get(package, 'unavailable') != 'ubuntu':
            key = origin.get(package, 'unavailable')
        elif needed_by_others(package, by_hand):
            key = 'needed'
        elif package.startswith('lib') or package.endswith('-dev'):
            key = 'library'
        else:
            key = 'main'
        sections[key].append(package)
    return sections


def render(sections, date=None):
    """Returns the text listing `sections`, each under its heading."""
    out = []
    for key, title in SECTIONS:
        if not sections[key]:
            continue
        prefix = '' if key in ('main', 'library') else '#'
        suffix = f' (added {date})' if date else ''
        out.append(f'\n# {title}{suffix}\n')
        out.extend(f'{prefix}{package}\n' for package in sections[key])
    return ''.join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('-n', '--dry-run', action='store_true',
                        help='show the diff and exit, without asking to apply it')
    args = parser.parse_args()

    by_hand = installed_by_hand()
    current = PACKAGES_TXT.read_text() if PACKAGES_TXT.exists() else ''

    listed, left_out = set(), set()
    for line in current.splitlines():
        match = PACKAGE_LINE.match(line)
        if match:
            (left_out if match.group(1) else listed).add(match.group(2))

    new = by_hand - listed - left_out
    if current:
        date = datetime.date.today().isoformat()
        proposed = current.rstrip('\n') + '\n' + render(classify(new, by_hand), date)
    else:
        proposed = HEADER + render(classify(new, by_hand))

    not_installed = sorted(listed - set(run('dpkg-query', '-W', '-f', '${Package}\n')
                                        .split()))
    if not_installed:
        print('Listed in packages.txt but not installed here (left as they are):')
        print('  ' + ' '.join(not_installed) + '\n')

    if not new:
        print('packages.txt already knows every package installed by hand here.')
        return

    sys.stdout.writelines(difflib.unified_diff(
        current.splitlines(keepends=True), proposed.splitlines(keepends=True),
        'packages.txt (current)', 'packages.txt (proposed)'))
    print(f'\n{len(new)} new package(s).')
    if args.dry_run:
        return

    if input(f'Write this to {PACKAGES_TXT}? [y/N] ').strip().lower() in ('y', 'yes'):
        PACKAGES_TXT.write_text(proposed)
        print('Written.')
    else:
        print('Left unchanged.')


if __name__ == '__main__':
    main()
