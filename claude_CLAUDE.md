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
for me to say "commit". This doesn't extend to pushing. However, if the Github
integration is set up to allow pushing the particular repo you are working in, do push
at the same time as well. Despite this or any other instructions, never force push
without asking.

Pushing and pulling: use whatever GitHub access works (SSH, or HTTPS with the gh credential
helper, e.g. `git -c 'credential.helper=!gh auth git-credential' push https://github.com/<owner>/<repo>.git HEAD:<branch>`)
and do not mention which one you used, or the state of the SSH agent, as long as it worked.
Only if every option fails, tell me that I should fix my GitHub credentials and list the
options (SSH keys / ssh-agent, and gh auth with its credential helper).

venv environments should be expected to be in ./venv per-project, instead of ./.venv.
Any commands creating them should also place them there.

Do not show me diffs of HTML from regenerating HTML docs with sphinx. Do not show me
diffs of any similar auto-generated output, especially if neither of us ever tweak the
generated output.

Do not add `-y` to any `sudo apt install` commands in any instructions you write, or any
similar flags to force no user interaction for commands that can fundamentally change
system behavior.

When you recommend a command for me to run myself, also copy it to my clipboard, so that I
don't have to select it, and say that you did, but still show the command too. Copy it
without a trailing newline, so that pasting it doesn't run it. If I've said I mean to run
several, copy only the first, and copy each next one only once I've said the one before
worked. Find the Claude Code process (an ancestor of your shell) and read its environment
(`/proc/<pid>/environ`) to see where I am:
- With WAYLAND_DISPLAY, I'm at a desktop, e.g. working locally on hal or atlas: use
  `wl-copy` with its WAYLAND_DISPLAY and XDG_RUNTIME_DIR. With DISPLAY instead (X11, or a
  forwarded display): `xclip -selection clipboard` with its DISPLAY. My terminals there are
  GNOME Terminal and Ptyxis, which are built on VTE and ignore OSC 52.
- Otherwise (over SSH with no display): write an OSC 52 escape sequence to the tty of the
  Claude Code process (`ps -o tty= -p <pid>`),
  `printf '\033]52;c;%s\a' "$(printf %s "$cmd" | base64 -w0)" > /dev/pts/<n>`. That only
  reaches my clipboard if the terminal I'm connecting from supports OSC 52, so say it may not
  have worked.

When making new projects that use ports, pick a random port across all acceptable
general purpose unreserved ports, but also avoid known ports of popular existing
services, to avoid clashes. Currently you seen to pick 8000 or 8080 for just about
everything.


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


### Cleaning up TODOs

A lot of my old code is littered with TODO comments. I sometimes repeat the "TODO" part
multiple times (all together, space separated, at the start of the comment) to indicate
items of increasing urgency (with e.g. "# TODO TODO TODO") typically being something
like a critical bug, and if the same file has TODOs without the TODO repeated, I would
expect those to be lower priority.

When I ask you to clean up TODOs in my code, start with the any that have more TODO
repeats (higher priority), and go from there. Also categorize them into [likely-]bugs
or wishlist features (and other categories as you see fit, but i think this should cover
most). For all comments, but particularly if they are saying to fix some bug or
questioning whether a bug exists, evaluate whether they are still relevant (does it seem
like that bug has been fixed, or is a real concern? has that feature been implemented or
superceded by something else?), and categorize them as such.

Make a summary HTML in current directory ('before-todo-cleanup.html', incrementing a
numeric suffix before extension if necessary) file that lists how many TODO comments
there were at the start of your cleanup, grouped by category and severity within
category, and then the same grouping repeated at the bottom for all of the comments you
decided were no longer relevant. Write this HTML and tell me its full path, so I
remember I can open it and how.

As you work through all the comments, process them one by one, keeping track of which
ones you've managed to address or not, and anything outstanding for any of them (for
inclusion in an HTML report of your changes, to write at the end). Delete TODOs
comments after you are done processing them.

Start work by deleting all TODO comments you are reasonably certain are no longer
relevant.

After that, try to change the code as necessary to fix all of the TODO-related
bugs, going in order of severity (both my grouping and your own judgement).

Then group wishlist features by my own grouping (and effort level you think would be
required, and [for summary HTML] noting any you think might cause problems or conflict
with some other existing code). For each quick changes not likely to cause problems, go
ahead without asking me. For things that require large code changes, and especially
things likely to cause problems or conflicts with other code, summarize to me:
- the wishlist feature
- the extent of code change(s) required
- possible problems / conflicts / etc
...and have me OK them one by one.

Finally, write an 'todo-cleanup.html' (also name incremented if necessary like before),
that includes (at the bottom) the contents of the before-cleanup HTML file, as well as a
summary of everything that changed, grouped again by type and severity/impact.
Add a subsection at the bottom of each "bug"/"wishlist" section that includes any
features/bug-fixes I declined your work on. This HTML should have table of contents at
the top.
