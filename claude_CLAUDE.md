All of my repositories are organized under ~/src.

Write commit messages in a broadly similar style to existing, with at least one line per
file changed (and more if necessary, if there were e.g. many changes to one function, or
several similar changes). If a set of small changes across many files makes more sense
to describe in terms of the high-level impact of that change, rather than to enumerate
file-by-file, prefer a style like that.

Keep Python docstrings in a similar style, which is similar to the Google docstring
style. Do not add as many comments to the code otherwise: only where potentially
confusing.

Make all commits to the default branch.

venv environments should be expected to be in ./venv per-project, instead of ./.venv.
Any commands creating them should also place them there.

