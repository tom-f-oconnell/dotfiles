
I added a `link_only` script, which will link all of the configuration files
without running the additional installation steps that `install` does.

Current steps to deploy only config links + VIM extensions on WSL:
1. See TODO.txt for notes on parts of WSL config I have not automated. 
   Changing the home folder is among the important such steps.
2. 
   ```
   git clone git@github.com:tom-f-oconnell/dotfiles --recurse-submodules
   cd dotfiles
   ./link_only
   ./install_vim_extensions.sh
   ```

[Upstream for this fork.](https://github.com/anishathalye/dotbot)

#### Machine-specific config

Anything specific to one machine (mount points, hardware names, etc) goes in
`~/.bash.local`, which is untracked and sourced by `~/.bashrc` in interactive shells.
`install` and `link_only` create it by copying `bash.local.template`, where everything
is commented out. The `vars` alias opens it for editing.

`./sync_bash_local.py` appends anything new in `~/.bash.local` to the template (commented
out, and not committed), and warns about anything in the template but not in
`~/.bash.local`, offering to add it there (also commented out). Commented-out code should
have no space after the `#`, and prose should have one, so the script can tell them apart.

#### Commands

Scripts meant to be run by name are in `bin/`, which `bashrc` adds to the end of `PATH`
(so they never shadow system commands of the same name).

- `bin/claude-sessions`: Claude Code sessions, here or on other hosts (over ssh, where it
  needs only python3 and Claude Code). `claude-list [host ...]` lists them (status and for
  how long, terminal, time since anyone typed there). `claude-peek <name|id> [host]` shows
  one read-only: status, terminal, working directory and git state, and its last few turns.
  `claude -r <id or name>` / `claude -c` / the `-r` picker, for a session still open in
  another terminal or the background, first offers to show, take over, close or fork it,
  rather than resume it twice (both would write one transcript without seeing each other's
  turns, and a later resume keeps only one side).

#### Packages

`packages.txt` lists the apt packages to install on a new machine, one per line. A line
`#name` is a package that is known but left out: comment a package out rather than
deleting it, or it will be proposed again. `./update_packages_txt.py` finds packages
installed by hand on this machine that the list doesn't know yet, shows the additions as
a diff, and writes them only after a yes (`-n` to only show the diff). To install the
list: `sudo apt install $(grep -v '^#' packages.txt)`.

#### Scheduled jobs

These are systemd user timers rather than cron jobs: they need no root, and their unit
files (in `systemd/`) are linked, and enabled, like any other config. They also run while
logged out, as long as linger is on for the user (`loginctl show-user $USER -p Linger`;
`loginctl enable-linger` turns it on, which may need an admin). `install` also starts them
right away; after `link_only`, they start from the next login, or run
`systemctl --user daemon-reload && systemctl --user start <name>.timer`.

- `bin/claude-prune-stale`: daily, `claude rm` for background Claude Code sessions that
  have exited (whatever their last status: done, needs input, failed) and been untouched
  for 7+ days (`$CLAUDE_PRUNE_DAYS` changes that). `claude-stale` lists what it would
  remove.

`systemctl --user list-timers` shows when each runs next, and `journalctl --user -u
<name>` what it did.

#### Claude Code status line

`claude_statusline.sh` puts the model and its effort level below the Claude Code prompt,
then bars for how much of the plan's 5-hour and weekly usage limits is used (and when each
resets), then any per-model weekly limits (e.g. Fable's), then as much of the session's
name as fits, and the vim mode at the right edge. The bars narrow as needed to fit the
rest but the name. It needs `jq` and `curl`. Usage only shows with a Pro / Max
/ Ultra login, after a session's first reply. The per-model limits come from the same place
as `/usage`, fetched at most every 2 minutes, so they can lag that much.

#### Claude Code settings

`claude_settings.json` is committed without the `model`, `effortLevel` and `modelSettings`
keys, which Claude Code rewrites on every `/model` and `/effort`: a git filter (in
`gitconfig`, applied by `.gitattributes`) strips them, so they stay in the working copy
but never show as changes. It needs `jq`; without it git warns and the keys show again.

#### Vim clipboard

In Vim, `,y` copies a Visual selection (or a motion) to the system clipboard, and `,v`
pastes from it. To copy w/ the terminal's Shift+drag selection instead, `,C` first
hides the fold/sign/number columns, so they aren't included.

Over SSH w/o X forwarding, Vim can still copy via OSC 52 (an escape sequence asking your
local terminal to set the clipboard), but only in terminals that support it (e.g. kitty,
WezTerm, foot, Alacritty, Ghostty, Windows Terminal), and NOT GNOME Terminal, Ptyxis, or
other VTE-based terminals. Vim's detection of this usually fails (incl. in tmux), so if
switching to such a terminal, uncomment `let g:osc52_force_avail = 1` in `vimrc`.

If `,y` doesn't reach the clipboard, check `:echo v:clipmethod` in Vim:
- `x11` or `wayland`: should work.
- `osc52`: only works if the terminal supports OSC 52 (see above). Also, `,v` can't
  paste from it (use the terminal's paste).
- `none`: Vim can't reach any clipboard (`,y` / `,v` only work within Vim). Check:
  - `vim --version | grep clipboard` shows `+clipboard`. On Ubuntu, install `vim-gtk3`
    (the `vim` and `vim-nox` packages lack it).
  - `echo $DISPLAY` is non-empty. Over SSH, connect w/ `ssh -Y` (the server needs
    `X11Forwarding yes` in `/etc/ssh/sshd_config`, and `xauth` installed).
  - In tmux, shells started before your latest attach can have a stale `DISPLAY`. Fix w/
    `eval "$(tmux show-env -s DISPLAY)"`, or open a new tmux window.

#### vim-plug

Vim plugins are managed by [vim-plug](https://github.com/junegunn/vim-plug) (kept here, in
`vim/autoload/plug.vim`; `:PlugUpgrade` updates it), and installed into `vim/bundle`
(ignored by git). `vpi` (`vim +PlugInstall +qall`) installs any missing, building
YouCompleteMe as it does (`install.py --clangd-completer`, set on its `Plug` line in
`vimrc`), which needs `build-essential cmake python3-dev`. `install_vim_extensions.sh`
installs those, then updates every plugin and re-runs all of their install steps. After a
Python upgrade, rebuild YouCompleteMe with `vim +'PlugInstall! YouCompleteMe' +qall`.

`:PlugClean` removes plugin directories no longer listed in `vimrc`.

#### Neovim

Neovim (`nvim`) uses the same `vimrc`, `vim/` and plugins as Vim, which stays the default
editor. `nvim/init.vim` (linked from `~/.config/nvim`) sources `vimrc`, which checks
`has('nvim')` where Neovim needs something different, and plugins only one editor uses are
listed with `Cond(has('nvim'))` (so `:PlugClean` in the other keeps them). The main
difference is markdown: Neovim renders it with
[render-markdown.nvim](https://github.com/MeanderingProgrammer/render-markdown.nvim)
(configured in `nvim/lua/markdown_render.lua`), which also wraps tables to fit the window.

`install_neovim.sh` (run by `install`) installs a nightly build in `~/.local/opt/nvim`,
because wrapping by displayed (not raw) width isn't in a release until 0.13. `-u` updates
it. It also makes a venv w/ `pynvim` for Neovim's Python provider (which YouCompleteMe
needs), in `~/.local/share/nvim/venv`.

Both keep swap files in `~/.local/state/nvim/swap` (Neovim's default), rather than next to
each file, so each warns about files open in the other.

Differences from Vim:
- YouCompleteMe works, but w/o its popups (signature help, hover), inlay hints or symbol
  search, which are Vim-only.
- Command history, marks and registers are kept separately (`~/.local/state/nvim/shada`).
- The clipboard (`,y` / `,v`) uses `xclip` w/ a display, else tmux's buffer (which tmux
  passes on via OSC 52), else OSC 52 if the terminal supports it.

#### multi-git-status

For now, just manually follow the instructions copied from [here](https://github.com/fboender/multi-git-status):

```
cd multi-git-status
PREFIX=~/.local make install
```
