# Migrating from movein.sh to packages.txt

Written 2026-11-13. Delete this file once the migration is done or dropped.

## Where things stand

`./install` runs `sudo ~/src/scripts/movein.sh` again. That step was commented out in
`install.conf.yaml` and is now re-enabled, so the next `./install` will ask for your sudo
password and then show one `apt install` prompt.

`movein.sh` was updated for Ubuntu 26.04 the same day:

- Its packages are installed by a single `apt install`, without `-y`, so it shows what it
  will do and asks once.
- It adds no PPAs.
- Every package it lists exists in 26.04. A dry run on atlas resolved cleanly (26 new
  packages, 6 upgrades).

It has not actually been run since the update. Only the package list was dry-run. The rest
of the script (making `~/src`, the SSH key check, `mk_key.sh`) is untested on 26.04.

## Reminder: check the new process

The plan is to replace `movein.sh`'s package list with `packages.txt`. Before switching,
check that the new process works for you:

1. **Prune `packages.txt`.** It is a first draft from atlas: 115 active packages and 35
   commented out. Comment out (`#name`) what you don't want on a new machine. Don't delete
   lines, or the script will propose those packages again. The "Libraries and headers"
   group is probably where most of the pruning is. The `unity-*` and `sssd-*` entries in
   the main group look like leftovers from release upgrades, not things you chose.
2. **Run `./update_packages_txt.py`** after installing something new by hand. Check that
   it proposes that package, that the diff reads sensibly, and that answering `n` leaves
   the file alone. It takes about 30 seconds.
3. **Try the install line** on a machine that is missing some of the list:
   `sudo apt install $(grep -v '^#' packages.txt)`
4. **Run it on another machine** (hal). The script only looks at the machine it runs on, so
   check that what it proposes there is what you'd want merged into the one list.

## If the new process works

- Replace the `movein.sh` step in `install.conf.yaml` with the `apt install` line above.
  The TODO comment above that step says the same.
- Decide what happens to the rest of `movein.sh` (making `~/src`, the SSH key check): move
  it into dotfiles, or leave `movein.sh` as a smaller script without a package list.
- Packages that `movein.sh` installs but `packages.txt` leaves out or lacks would need
  adding: check `wl-clipboard`, `libnotify-bin`, `gparted`, `autofs`, `aptitude`, `iotop`,
  `htop`, `bash-completion`, `grip` and `python3-tk`.

## Loose ends

- The old `git-core` PPA is still configured on atlas, though disabled, so it has no
  effect. To remove it:
  `sudo rm /etc/apt/sources.list.d/git-core-ubuntu-ppa-focal.sources /etc/apt/sources.list.d/git-core-ubuntu-ppa-focal.list.save`
- Docker and TeamViewer are installed on atlas, but their repositories are disabled there,
  so `packages.txt` lists them as left out. A new machine needs those repositories set up
  separately. The same goes for Chrome and Claude Desktop.
- `ui_settings.sh` was rewritten for GNOME on 2026-11-13. Three things in it are commented
  out, to re-evaluate: the Alt+M / Alt+K / Alt+J volume keys, the faster key repeat, and
  the Ctrl+Q hotkey (whose script needs X11). The favorite apps line is also commented
  until those apps are all set up.
