
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

#### Vundle

To remove all directories Vundle adds to `vim/bundle`:
```
cd vim/bundle
# https://stackoverflow.com/questions/9314365
# Add -n flag to see what it would remove first.
git clean -f -f -d -x
```
The above might cause some issues. Not sure.

To only install Vundle plugins, without trying to do the YouCompleteMe
compilation that `install_vim_extensions.sh` does:
```
# I also have this aliased as `vpi`
vim +PluginInstall +qall
```

#### multi-git-status

For now, just manually follow the instructions copied from [here](https://github.com/fboender/multi-git-status):

```
cd multi-git-status
PREFIX=~/.local make install
```
