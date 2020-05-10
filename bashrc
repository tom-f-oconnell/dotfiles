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

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

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
alias l='ls -CF'

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

# Alias definitions.
# Some depend on completion, so important this comes after bash_completion
# /usr/share/doc/bash-doc/examples in the bash-doc package (example aliases).
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
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
#if [ -f /opt/ros/kinetic/setup.bash ]; then
#  source /opt/ros/kinetic/setup.bash
#fi
#if [ -f $HOME/catkin/devel/setup.bash ]; then
#  source $HOME/catkin/devel/setup.bash
#fi
## I think this can conflict with the devel environment.
##source $HOME/catkin/install/setup.bash

# TODO better way to manage python path to include my modules nested within src?

if [ -d "$HOME/src/SutterMP285" ]; then
  export PYTHONPATH="${PYTHONPATH}:$HOME/src/SutterMP285"
fi

if [ -d "$HOME/catkin/src/multi_tracker/multi_tracker_analysis" ]; then
  export PATH="$PATH:$HOME/catkin/src/multi_tracker/multi_tracker_analysis"
fi

if [ -d "$HOME/src/scripts" ]; then
  export PATH="$HOME/src/scripts:$PATH"
fi

if [ -f "$HOME/.variables" ]; then
    . "$HOME/.variables"
fi

# As alternative to ssh-agent being started on login, see:
# https://stackoverflow.com/questions/17846529#24347344

# see Litmus' answer
# https://stackoverflow.com/questions/18880024/start-ssh-agent-on-login
SSH_ENV="$HOME/.ssh/environment"

# was trying to eval / exec this (not sure whether either could work),
# so we can grep the correct command when grepping ps output.
# (otherwise, adding the -t <NNNN> flag to ssh-agent would require
# changing the grep pattern to also include this flag...)
# (for now, just going to remove the end of line restriction?)
#AGENT_CMD="/usr/bin/ssh-agent -t 28800"

# TODO TODO TODO AddKeysToAgent in config seems to be working, but now i only
# have access to that agent in one terminal (doesn't work in a new one).
# fix!!!!! (looks like multiple agents are being created, b/c 
# SSH_AGENT_PID differs in each shell)
function start_agent {
    # TODO modify this so the "succeeded" message is on the same line
    # (use printf?)
    echo "Initialising new SSH agent..."
    # The -t flag should set the lifetime of keys within the agent.
    # 28800 is how many seconds there are in 8 hours.
    /usr/bin/ssh-agent -t 28800 | sed 's/^echo/#echo/' > "${SSH_ENV}"
    #/usr/bin/ssh-agent | sed 's/^echo/#echo/' > "${SSH_ENV}"
    #exec ${AGENT_CMD} | sed 's/^echo/#echo/' > "${SSH_ENV}"
    echo succeeded
    chmod 600 "${SSH_ENV}"
    . "${SSH_ENV}" > /dev/null
    # TODO delete some of the above (up to everything besides just starting
    # ssh-agent), if that was only necessary for this ssh-add call).
    # trying to have 'AddKeysToAgent yes' in ~/.ssh/config do this now, so that
    # the password is only needed the first time the key is used, not on login
    # https://superuser.com/questions/325662
    # /how-to-make-ssh-agent-automatically-add-the-key-on-demand
    #/usr/bin/ssh-add;
}

# Source SSH settings, if applicable
# TODO only do this from first time i use ssh / git, not from first shell?
# TODO also consider using user Micah's answer to same question, which should
# kill ssh_agent when no more bash processes (?)
if [ -f "${SSH_ENV}" ]; then
    . "${SSH_ENV}" > /dev/null
    #ps ${SSH_AGENT_PID} doesn't work under cywgin

    # see comment near AGENT_CMD above
    #ps -ef | grep ${SSH_AGENT_PID} | grep ssh-agent$ > /dev/null || {
    ps -ef | grep ${SSH_AGENT_PID} | grep ssh-agent > /dev/null || {
        start_agent;
    }
else
    start_agent;
fi

# TODO why did i want this / on which machine did/do i have the util dir again?
# (not on 2019 blackbox, but maybe laptop or a lab machine?)
# TODO maybe don't do this
# https://unix.stackexchange.com/questions/17715/
# how-can-i-set-all-subdirectories-of-a-directory-into-path/17856#17856
#export PATH="$( find $HOME/src/dotfiles/util/ -type d -printf "%p:" )$PATH"

# For rdkit cheminformatics library
export RDBASE=$HOME/src/rdkit
export LD_LIBRARY_PATH=$RDBASE/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$RDBASE:$PYTHONPATH

# What did this flag do again?
export MATLAB_USE_USERWORK=1

if [ -f "/opt/openfoam6/etc/bashrc" ]; then
    source /opt/openfoam6/etc/bashrc
fi

if [ -x "$(command -v direnv)" ]; then
    # Despite (trying to) move this below bashrc conda section, direnv still
    # faces problems trying to run conda commands (conda complains about not be
    # setup correctly, at least for deactivate). Currently using workaround in:
    # https://github.com/conda/conda/issues/7980
    # ...which is (in .envrc files) sourcing a specific conda config file before
    # any conda commands
    eval "$(direnv hook bash)"
fi

# TODO TODO TODO possible to make a conda init block that will work across all
# my conda installations? otherwise how to deal w/ the fact that conda wants to
# put this in bashrc, but i want to manage bashrc in source control, in a
# deployment independent manner...

# TODO deal w/ anaconda sections in a deployment-conda-version-specific manner
# if necessary (try to avoid though) (may also want to delete any commented
# blocks, in case conda still detects and tries to manage it)
# added by Anaconda3 2018.12 installer
## >>> conda init >>>
## !! Contents within this block are managed by 'conda init' !!
#__conda_setup="$(CONDA_REPORT_ERRORS=false '/home/tom/anaconda3/bin/conda' shell.bash hook 2> /dev/null)"
#if [ $? -eq 0 ]; then
#    \eval "$__conda_setup"
#else
#    if [ -f "/home/tom/anaconda3/etc/profile.d/conda.sh" ]; then
#        . "/home/tom/anaconda3/etc/profile.d/conda.sh"
#        CONDA_CHANGEPS1=false conda activate base
#    else
#        \export PATH="/home/tom/anaconda3/bin:$PATH"
#    fi
#fi
#unset __conda_setup
## <<< conda init <<<

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/tom/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/tom/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/tom/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/tom/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

# Moved this stuff after conda setup, so that (in a fresh shell) conda isn't
# doing its setup with a potentially-error-full custom startup script of mine.
# One example of a problem: the eval in the conda setup means if
# usercustomize.py prints anything, it will also be eval'd.
if [ -d $HOME/src/scripts ]; then
  export PATH="$HOME/src/scripts:$PATH"
  export PYTHONPATH="${PYTHONPATH}:$HOME/src/scripts/python_startup"
  # This is a variable read by my scripts/python_startup/excepthook.py module.
  export PYMISTAKE_DEBUG_UNCAUGHT="1"
fi

# TODO TODO only do all the direnv + conda hacks if BOTH are installed

# Using existence of this file to since show_virtual_env()
# can make files, but does not seem able to export environment variables
# back to parent shell... (not sure if there's some way of doing this that
# makes more sense...)
# TODO using tty to prevent collisions from diff terminals ok?
# tmux or stuff like that cause probs?
export DIRENV_CONDA_FLAG_FILE_PREFIX="direnv_conda_env_"
# TODO cases where hardcoding /tmp like this will hurt compat?
DIRENV_PS1_UPDATE_FLAG_FILE="/tmp/${DIRENV_CONDA_FLAG_FILE_PREFIX}"
# Also used to name the anaconda activate/deactivate hook scripts.
export DIRENV_TTYSTR=`tty | sed -e "s:/dev/::" -e "s/\//_/g"`
DIRENV_PS1_UPDATE_FLAG_FILE+="${DIRENV_TTYSTR}"
# In case a previous file from same TTY ID (IDs presumably must have been
# recycled) was somehow not deleted properly.
rm -f "${DIRENV_PS1_UPDATE_FLAG_FILE}"
export DIRENV_PS1_UPDATE_FLAG_FILE

export DIRENV_CONDA_SCRIPT_PREFIX="direnv_ps1_fix_${DIRENV_TTYSTR}.sh"
pass_ps1_ctrl_to_conda_str() {
    # Just in case functions to update PS1 would for some reason not be run...
    if [[ -f "$DIRENV_PS1_UPDATE_FLAG_FILE" ]]; then
        local conda_prefix_to_deact=`cat $DIRENV_PS1_UPDATE_FLAG_FILE`

        local activate_sh=""
        local deactivate_sh=""
        if [[ -n "$conda_prefix_to_deact" ]]; then
            activate_sh="$conda_prefix_to_deact/etc/conda/activate.d/"
            activate_sh+="$DIRENV_CONDA_SCRIPT_PREFIX"
            deactivate_sh="$conda_prefix_to_deact/etc/conda/deactivate.d/"
            deactivate_sh+="$DIRENV_CONDA_SCRIPT_PREFIX"
        fi

# TODO fix indentation to tabs and tab out heredoc if possible
cat <<EOF
rm -f "$DIRENV_PS1_UPDATE_FLAG_FILE"
# The (terminal+env)-specific conda hooks that would delete the above
# flag file upon manual conda activate / deactivate.
rm -f "$activate_sh"
rm -f "$deactivate_sh"
EOF
    else
        printf ""
    fi
}
pass_ps1_ctrl_to_conda() {
    eval "$(pass_ps1_ctrl_to_conda_str)"
}
export -f pass_ps1_ctrl_to_conda_str
export -f pass_ps1_ctrl_to_conda
trap pass_ps1_ctrl_to_conda EXIT

# TODO also test case going from having direnv_dir to it being empty/unset
show_virtual_env() {
  if [[ -n "$DIRENV_DIR" ]]; then
    # I tried to adapt the wiki example which explains how to get this to work
    # for EITHER conda or virtualenv, but this should hopefully get it to
    # work with both, giving precedence for conda.
    if [[ -n "$CONDA_DEFAULT_ENV" && -f "$DIRENV_PS1_UPDATE_FLAG_FILE" ]]; then
      echo "($(basename $CONDA_DEFAULT_ENV)) "
    elif [[ -n "$VIRTUAL_ENV" ]]; then
      echo "($(basename $VIRTUAL_ENV)) "
    else
      return 0
    fi
  fi
}
export -f show_virtual_env
PS1='$(show_virtual_env)'$PS1
# TODO fix how above doesn't seem to let conda clear PS1 in case where
# base env is enabled by default (base) comes before $(show_virtual_env).
# if that's the source of the problem, maybe prepend it to backup ps1
# in POST_COMMAND every time, so conda can so its stuff first?
# TODO and make my envrc stuff not conflict w/ systems configured to have base
# enabled by default (should probably never / only under certain circumstances
# deactivate then. can prob use conda to lookup config as part.)

# So it can be restored if interaction between direnv and conda
# screw up PS1.
PS1_BACKUP="$PS1"
reset_ps1() {
    local previous_exit_status=$?;
    if ! [[ -n "$CONDA_DEFAULT_ENV" || "$PS1" = "$PS1_BACKUP" ]]; then
        export PS1="$PS1_BACKUP"
    fi
    return $previous_exit_status
}
export -f reset_ps1
PROMPT_COMMAND+=" reset_ps1"
# TODO TODO may have to investigate circumstances where the above
# (+ other direnv stuff) causes direnv to add serious lag to commands


# Re-defining the command_not_found_handle, which should be defined in
# /etc/bash.bashrc
# Using information from these two posts:
# https://superuser.com/questions/787424
# https://stackoverflow.com/questions/1203583
eval "$(echo "orig_command_not_found_handle()"; declare -f command_not_found_handle | tail -n +2)"
command_not_found_handle() {
    if [ -f "$1" ]; then
        echo "cmd not found. editing file in current directory."
        vi "$1"
    else
        orig_command_not_found_handle "$@"
    fi
}

# https://stackoverflow.com/questions/38859145
if grep -q Microsoft /proc/version; then
    # Since the $HOME I set by modifying that passwd file is still better as the
    # parent of this, but virtually all times I open a WSL terminal, I'll want
    # to go here.
    cd ~/src
fi

