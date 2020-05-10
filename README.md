
I added a `link_only` script, which will link all of the configuration files
without running the additional installation steps that `install` does.

Current steps to deploy only config links + VIM extensions on WSL:
1. See TODO.txt for notes on parts of WSL config I have not automated. 
   Changing the home folder is among the important such steps.
2.
```
git clone git@github.com:tom-f-oconnell/dotfiles
cd dotfiles
./link_only
./install_vim_extensions.sh
```

Upstream for this fork is [dotbot](https://github.com/anishathalye/dotbot).

