# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# All HISTFILE section from: https://stackoverflow.com/questions/9457233
# Leaving these empty makes the history size unbounded.
export HISTSIZE=
export HISTFILESIZE=
# Change the file location because certain bash sessions truncate .bash_history
# file upon close. http://superuser.com/questions/575479
export HISTFILE=~/.bash_eternal_history
# Force prompt to write history after every command.
# http://superuser.com/questions/20900/bash-history-loss
PROMPT_COMMAND="history -a; $PROMPT_COMMAND"

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
# TODO some reason i don't want this as the default?
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
        # We have color support; assume it's compliant with Ecma-48
        # (ISO/IEC-6429). (Lack of such support is extremely rare, and such
        # a case would tend to support setf rather than setaf.)
        color_prompt=yes
    else
        color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# Not sure if I did something to enable what seemed to be the previous behavior
# of having limited vim-like behavior in bash (navigation but not e.g. deleting
# a word), but this should also enable vim-like editing
# TODO possible to make the vim command 'daw' work? seems like it doesn't work
# with or without this line (and idk how i already got vim line-editing, but
# 'vi' was not in $SHELLOPTS without this line, so it's not through this,
# because set adds to that variable) ('dw' does work though, either way)
set -o vi

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# https://stackoverflow.com/questions/38859145
if grep -q Microsoft /proc/version; then
    # To silence errors about inaccessible symlinks.
    # https://unix.stackexchange.com/questions/445917
    alias ls='ls --color=tty 2>/dev/null'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
#alias l='ls -CF'
alias l='ls'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

export EDITOR="/usr/bin/vi"

# Really just wanted vi tab completion to go right to the <x>.py rather than
# <x>.egg-info/ for my (installed) single-file modules.
# But couldn't figure out a way to modify the vi command tab completion
# specificaly (though it seems possible).
# https://stackoverflow.com/questions/32189015
export FIGNORE=".egg-info"

# At one point, I decided I needed to either comment anaconda init or ROS init,
# lest some conflict emerge (which was what again?).
# See these two posts for discussion of problem + possible
# (though a bit complicated) workaround:
# https://github.com/conda/conda/issues/7980
# https://gist.github.com/StefanFabian/17fa715e783cd2be6a32cd5bbb98acd9
# TODO figure out if this is still necessary w/ latest anaconda
# (i.e. if I call conda deactivate, can I then use ROS normally?)
# TODO TODO TODO test on systems that actually have both installed
# (blackbox has no ROS right now, and it can't get kinetic anyway cause 18.04)
# TODO either way, maybe have my config management (dotbot) copy a separate
# config file that has a flag saying which of the two the machine should be used
# for (prompt / then edit to correct value, the system-specific value outside
# source control)?
# For now, I'm just going to opt to not have the conda environment loaded by
# default, as: https://stackoverflow.com/questions/54429210
# TODO maybe rename to indicate usage for toggling between conda / ros. and delete if i
# can find a way to get them to play nicely w/o this manual switching.
ENABLE_ROS=false
if [ "${ENABLE_ROS}" = true ]; then
    if [ -f /opt/ros/kinetic/setup.bash ]; then
      source /opt/ros/kinetic/setup.bash
    fi
    if [ -f /opt/ros/melodic/setup.bash ]; then
      source /opt/ros/melodic/setup.bash
    fi
    if [ -f /opt/ros/noetic/setup.bash ]; then
      source /opt/ros/noetic/setup.bash
    fi

    if [ -f $HOME/catkin/devel/setup.bash ]; then
      source $HOME/catkin/devel/setup.bash
    fi
    ## I think this can conflict with the devel environment.
    #source $HOME/catkin/install/setup.bash

    # Options for ROS logging output, for debugging.
    # http://wiki.ros.org/rosconsole#Console_Output_Formatting
    # default is equivalent to '[${severity}] [${time}]: ${message}'
    export ROSCONSOLE_FORMAT='[${severity}] [${time}] ${node}: ${message}'
    #export ROSCONSOLE_FORMAT='[${severity}] [${time}] (${file}:${line}): ${message}'

    # For playing around with ROS turtlebot simulations.
    # https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros
    export TURTLEBOT3_MODEL=burger
fi

# TODO better way to manage python path to include my modules nested within src?

# Using a subshell function here to cd safely
function submodule_paths() (
    if [ "$#" -ne 1 ]; then
        echo "wrong number of arguments to submodule_paths. expect 1 directory."
        return 2
    fi
    if ! [ -d "$1" ]; then
        echo "argument to submodule_paths was not a directory!"
        return 1
    fi
    cd "$1"
    # Using '|' intead of '/' as sed separator character, to not need to worry
    # about the '/'s that are in the $1 path
    git submodule status | awk '{print $2}' | sed "s|.*|$1/&|"

    # TODO maybe also loop over the lines that would be printed, and check each
    # is a real directory, in case there are some weird cases where they aren't?
    # (like in case submodules are still listed in git metadata but not on disk)
)

# TODO maybe return to doing something like this for things in ~/src/scripts
# (or search some whitelist / submodules in that thing?)
# https://unix.stackexchange.com/questions/17715/17856#17856
#export PATH="$( find $HOME/src/dotfiles/util/ -type d -printf "%p:" )$PATH"

MY_SCRIPTS_PATH="$HOME/src/scripts"
if [ -d "$MY_SCRIPTS_PATH" ]; then
    export PATH="$PATH:$MY_SCRIPTS_PATH"

    for submodule_path in $(submodule_paths "$MY_SCRIPTS_PATH")
    do
        #echo "adding submodule at $submodule_path to PATH"

        # TODO maybe some kind of assert that the directory exists?

        # The output of my `submodule_paths` fn should contain *absolute* paths.
        export PATH="$PATH:$submodule_path"
    done
fi

# As alternative to ssh-agent being started on login, see:
# https://stackoverflow.com/questions/17846529#24347344

# see Litmus' answer
# https://stackoverflow.com/questions/18880024/start-ssh-agent-on-login
SSH_DIR="$HOME/.ssh"
SSH_ENV="$SSH_DIR/ssh-agent-env"

function start_agent {
    printf "Initialising new SSH agent... "
    # The -t flag should set the lifetime of keys within the agent.
    # 28800 is how many seconds there are in 8 hours.
    /usr/bin/ssh-agent -t 28800 | sed 's/^echo/#echo/' > "${SSH_ENV}"
    echo succeeded
    chmod 600 "${SSH_ENV}"
    . "${SSH_ENV}" > /dev/null
    # TODO delete some of the above (up to everything besides just starting
    # ssh-agent), if that was only necessary for this ssh-add call).

    # NOTE: this requires 'AddKeysToAgent yes' in ~/.ssh/config, since we aren't
    # ssh-add'ing here anymore (to only require typing password on ssh attempt, not on
    # first shell opening). this should be configured as part of my dotfiles setup.
}

if [ -d "${SSH_DIR}" ]; then
    # TODO also consider using user Micah's answer to same question, which
    # should kill ssh_agent when no more bash processes
    if [ -f "${SSH_ENV}" ]; then
        . "${SSH_ENV}" > /dev/null
        #ps ${SSH_AGENT_PID} doesn't work under cywgin

        # Not also including '$' end of line as part of grep (contra the SO answer this
        # came from), because my invocation of ssh-agent has trailing arguments.
        ps -ef | grep ${SSH_AGENT_PID} | grep ssh-agent > /dev/null || {
            start_agent;
        }
    else
        start_agent;
    fi
fi

# TODO move to a non-site-specific file just setting variables (maybe .profile?)
# "so that userpath will be used as the startup folder"
export MATLAB_USE_USERWORK=1

if [ -x "$(command -v direnv)" ]; then
    # Despite (trying to) move this below bashrc conda section, direnv still
    # faces problems trying to run conda commands (conda complains about not be
    # setup correctly, at least for deactivate). Currently using workaround in:
    # https://github.com/conda/conda/issues/7980
    # ...which is (in .envrc files) sourcing a specific conda config file before
    # any conda commands
    eval "$(direnv hook bash)"
fi

# TODO possible to make a conda init block that will work across all my conda
# installations? otherwise how to deal w/ the fact that conda wants to put this in
# bashrc, but i want to manage bashrc in source control, in a deployment independent
# manner...

# TODO see if https://github.com/rickstaa/.ros_conda_wrapper has anything of
# value (though unclear whether i'm supposed to still include conda and / or ROS
# lines of my own in bashrc, and if so, how to order everything)

# TODO deal w/ anaconda sections in a deployment-conda-version-specific manner
# if necessary (try to avoid though) (may also want to delete any commented
# blocks, in case conda still detects and tries to manage it)

# TODO at least test this on a machine without conda, and maybe refactor so this block
# only gets called if the $HOME/anaconda3[/bin/conda] dir[/executable] exist?

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/toor/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/toor/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/toor/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/toor/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

# Moved this stuff after conda setup, so that (in a fresh shell) conda isn't
# doing its setup with a potentially-error-full custom startup script of mine.
# One example of a problem: the eval in the conda setup means if
# usercustomize.py prints anything, it will also be eval'd.
PYMISTAKE_PATH=$HOME/src/pymistake
if [ -d $PYMISTAKE_PATH ]; then
  export PYTHONPATH="${PYTHONPATH}:${PYMISTAKE_PATH}"
  #export PYMISTAKE_DEBUG="1"
fi

## TODO also test case going from having direnv_dir to it being empty/unset
#show_virtual_env() {
#  if [[ -n "$DIRENV_DIR" ]]; then
#    # I tried to adapt the wiki example which explains how to get this to work
#    # for EITHER conda or virtualenv, but this should hopefully get it to
#    # work with both, giving precedence for conda.
#    if [[ -n "$CONDA_DEFAULT_ENV" && -f "$DIRENV_PS1_UPDATE_FLAG_FILE" ]]; then
#      echo "($(basename $CONDA_DEFAULT_ENV)) "
#    elif [[ -n "$VIRTUAL_ENV" ]]; then
#      echo "($(basename $VIRTUAL_ENV)) "
#    else
#      return 0
#    fi
#  fi
#}

show_virtual_env() {
  if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "($(basename $CONDA_DEFAULT_ENV))"
  fi
}
export show_virtual_env
PS1='$(show_virtual_env) '$PS1

# TODO TODO extend this to echo variables if their name is to be evaluated
# by itself (or maybe have it `declare -p <variable-name>`?)

# Re-defining the command_not_found_handle, which should be defined in
# /etc/bash.bashrc
# Using information from these two posts:
# https://superuser.com/questions/787424
# https://stackoverflow.com/questions/1203583
# MINGW didn't like these.

# TODO maybe just put this in an "if" checking not windows?
# or will it still cause a syntax error on windows even if it wouldn't be run?

#eval "$(echo "orig_command_not_found_handle()"; declare -f command_not_found_handle | tail -n +2)"
#command_not_found_handle() {
#    if [ -f "$1" ]; then
#        echo "cmd not found. editing file in current directory."
#        vi "$1"
#    else
#        orig_command_not_found_handle "$@"
#    fi
#}

# https://stackoverflow.com/questions/38859145
if grep -q Microsoft /proc/version; then
    # Since the $HOME I set by modifying that passwd file is still better as the
    # parent of this, but virtually all times I open a WSL terminal, I'll want
    # to go here.
    cd ~/src

    # This should allow matplotlib to work in WSL,
    # after following other steps here:
    # https://stackoverflow.com/questions/43397162
    export DISPLAY=localhost:0.0
fi

# TODO fix how mingw (git bash) colors seem screwed up by my ~/.bashrc or other
# settings (the colors of the prompt mostly)

# Delete me. Just trying to improve visibility of directories on windows
# (including in prompt ideally).
# (not working...)
#export LS_COLORS=$(echo $LS_COLORS | sed 's/di=01;34/di=01;94/g')

if [ -d "$HOME/src/rdkit" ]; then
    # For rdkit cheminformatics library
    export RDBASE=$HOME/src/rdkit
    export LD_LIBRARY_PATH=$RDBASE/lib:$LD_LIBRARY_PATH
    export PYTHONPATH=$RDBASE:$PYTHONPATH
fi

if [ -d "$HOME/src/SutterMP285" ]; then
  export PYTHONPATH="${PYTHONPATH}:$HOME/src/SutterMP285"
fi

if [ -f "/opt/openfoam6/etc/bashrc" ]; then
    source /opt/openfoam6/etc/bashrc
fi

if [ -d "/usr/local/nrn/x86_64/bin" ]; then
    export PATH="$PATH:/usr/local/nrn/x86_64/bin"
fi

if [ -d "$HOME/arduino-cli/bin" ]; then
    # Made this directory manually for arduino-cli
    export PATH="$PATH:$HOME/arduino-cli/bin"
fi

# For https://github.com/hoijui/ReZipDoc
if [ -d "$HOME/src/ReZipDoc/scripts" ]; then
    export PATH="$HOME/src/ReZipDoc/scripts:$PATH"
fi

# To explicitly specify which FreeCAD to use for my freecad_finder Python
# library, which helps you add FreeCAD libraries to Python sys.path for use in
# standalone scripts. Not used by FreeCAD itself.
export FREECAD_EXECUTABLE=$HOME/src/FreeCAD/build/bin/FreeCAD

if [ -f "$HOME/.variables" ]; then
    . "$HOME/.variables"
fi

# Alias definitions.
# Some depend on completion, so important this comes after bash_completion
# /usr/share/doc/bash-doc/examples in the bash-doc package (example aliases).
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# At least this first line seems necessary to use
# `xdg-mime default <x>.desktop <mimetype>` for desktop files under
# ~/.local/share/applications
# Adapted from https://unix.stackexchange.com/questions/637685 and
# https://askubuntu.com/questions/538526
# TODO though check again whether it was actually necessary to set this before that
# xdg-mime default ... command above worked
# TODO also look into setting this variables in perhaps a more appropriate place, as
# mentioned in the other answer in the second link above and in this post it references:
# https://superuser.com/questions/365847
export XDG_DATA_HOME=${XDG_DATA_HOME:="$HOME/.local/share"}
export XDG_CONFIG_HOME=${XDG_CONFIG_HOME:="$HOME/.config"}
export XDG_CACHE_HOME=${XDG_CACHE_HOME:="$HOME/.cache"}

