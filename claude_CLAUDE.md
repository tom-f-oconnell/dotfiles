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

Commit finished changes without asking me first (once tests pass), rather than waiting
for me to say "commit". This doesn't extend to pushing.

venv environments should be expected to be in ./venv per-project, instead of ./.venv.
Any commands creating them should also place them there.

Do not show me diffs of HTML from regenerating HTML docs with sphinx. Do not show me
diffs of any similar auto-generated output, especially if neither of us ever tweak the
generated output.


### READMEs

READMEs should be concise, starting with only a ~2-3 sentence summary of the project,
with the sections following grouped in roughly the following order:
1. Details relevant to install and use the software for the most tech-averse/illiterate
   users.
2. Details revevant to configure/use the software for power users.
3. Minimal details to setup and run the software for a dev environment (and only if
   meaningfully distinct from general setup instructions above). Only at the bottom of
   this section show how to run tests and then how to build any docs, if applicable.
   If testing is orchestrated through some other project (i.e. olfsysm is mostly tested
   via al_analysis, currently), mention that with a link here.

Bigger picture design decisions for the software should be consolidated into a separate
`design.md`, with a link to this (describing what it's for) in the dev section (between
initial setup and the tests/docs part). If many details are needed about fine grained
usage of the software, split this off into a separate `usage.md`, and link towards the
top of the power user section of the docs. However, try to keep usage instructions
concise and just in the README.

