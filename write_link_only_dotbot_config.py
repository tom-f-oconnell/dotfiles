#!/usr/bin/env python3

"""
Writes dotbot config YAML with all of the shell directives removed, to not
invoke expensive scripts after just adding a new config file to link.
"""

import sys
from os import stat
from os.path import join, split, isfile, exists
import re

# Requires PyYAML
import yaml


def isempty(fname):
    return stat(fname).st_size == 0


def config_filename():
    install_script = join(split(__file__)[0], 'install')
    assert isfile(install_script)
    with open(install_script, 'r') as f:
        data = f.read()

    match = re.search('^\s*CONFIG="([^$]+)"$', data, re.MULTILINE)
    fname = match.group(1)
    assert isfile(fname) and not isempty(fname)
    return fname


def write_modified_config(path):
    # mktemp does make the file, so can't assert it doesn't `exists`
    assert not exists(path) or isempty(path), \
        'expected a temporary file. should not exist or be empty.'

    config_fname = config_filename()
    with open(config_fname, 'r') as f:
        # Will be a list of dicts, where the keys are one of those in
        # `expected_keys` below.
        data = yaml.load(f)

    expected_keys = {'defaults', 'clean', 'shell', 'link' }
    new_data = []
    for d in data:
        keys = set(d.keys())
        # At least testing w/ my config so far, all dicts only have one key.
        # If that's not generally the case, would need more work.
        assert len(keys) == 1, 'see comment above'
        key = keys.pop()
        assert key in expected_keys
        if key != 'shell':
            new_data.append(d)

    with open(path, 'w') as f:
        yaml.dump(new_data, f)


def main():
    # First element is the name of the script, second should be a path to write
    # the modified config file to.
    assert len(sys.argv) == 2
    path = sys.argv[1]
    write_modified_config(path)


if __name__ == '__main__':
    main()

