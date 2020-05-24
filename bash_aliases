
# opens a file in Fiji, and appends current directory before argument so Fiji 
# doesn't freak out
function fp() {
    $HOME/Fiji.app/ImageJ-linux64 $(pwd)/$1
}

# TODO some function / alias to remind me to use shortcuts for git commands
# if i call the long forms. general approach, for any aliased commands?
function g() {
    git commit -am "$1"
}

# for aliases where the arguments should go in the middle
function ltx() {
    FILE=$1
    PREFIX=${FILE%%.*}
    latex $1
    RET=$?
    rm "$PREFIX.log"; rm "$PREFIX.aux"
    if [ $RET -eq 0 ]
    then
        dvipdfm $PREFIX.dvi; rm "$PREFIX.dvi"
        xdg-open $PREFIX.pdf
    fi
}

function gtest() {
    mkdir test
    cd test
    # TODO only continue if worked / warn
    cp -i ~/src/gtest_template/test/* .
    # TODO only continue if in git repo
    git submodule add git@github.com:google/googletest.git gtest
    #GITROOT=`git rev-parse --show-toplevel`
    #pushd .
    #cd $GITROOT
    # i guess you don't actually need to do these two when you first add it?
    # just on later clones? and is updated needed? (need to make a new commit if
    # it changes?)
    #git submodule init
    #git submodule update
    #popd
}

function _check_one_nonexistant_dir_arg() {
    if [ "$#" -ne 1 ]; then
        echo "Usage: $0 DIRECTORY_TO_MAKE" >&2
        return 1
    elif [ -d "$1" ]; then
        echo "directory '$1' already exists"
        return 1
    fi
    return 0
}

function pyp() {
    if _check_one_nonexistant_dir_arg "$@"; then
        mkdir $1
        cd $1
        echo "assuming python3 is installed (venv not available otherwise)"
        echo "layout python-venv" > .envrc
        # TODO maybe conda deactivate first (if applicable)?
        direnv allow
        git init
    fi
}

alias r=". ~/.bashrc"

# TODO TODO alias mv to some function that first tries git mv, then mv if not
# in a git repo
alias ga='git add'
alias gaa='git add --all'
alias gca='git commit -am'
alias gp='git push --follow-tags'
alias gpr='git pull --rebase'
# git "[i]nfo"
alias gi='git status'
alias gl='git log'

# wait, what is second argument for again? why not intervening flash?
clone_fn() {
	echo "Argument 1: $1"
	echo "Argument 2: $2"
	git clone git://github.com/$1 $2
}
alias c="clone_fn"
# TODO this this. if not working, get something w/ same fn working
# would this work?
#alias ct="clone_fn tom-f-oconnell/"

alias wow='wine /media/tb/Games/wow-4.3.4/wow_434.exe 1>/dev/null 2>/dev/null &'

alias lmms='$HOME/src/lmms/build/lmms'
alias lac='LAC'

#alias plots='scp tom@eftm.duckdns.org:~/lab/hong/src/*html .'
alias fiji='$HOME/Fiji.app/ImageJ-linux64'

# TODO get vim formatting (e.g. gq) to work with bash comments (recomment new
# lines) + string handling + command breaking

alias scpr='scp -r'
alias sshx='ssh -X'
# TODO what was this for again? forcing use of password? but why?
alias snk='ssh -o PubkeyAuthentication=no'
#alias atty='stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost
#-onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon
#-crtscts -hupcl'

alias x='xdg-open'
alias rs='rsync -auvP'
alias cpconf='cp $HOME/catkin/src/multi_tracker/examples/sample_data/config_20160412_134708_N1.py .; mv config_20160412_134708_N1.py config_`ls -p | grep bag | sed s/_delta_video.bag//g`.py'


# TODO actually re-setup my windows ~/.bash[rc/_aliases], so it works for git
# bash (in addition to still working for WSL)

# Trying to detect Git Bash on Windows, to alias python so it works on
# my Windows 10 setup, using the Python 3.8 that I think I installed from
# the Windows store?
if [[ "$(uname)" =~ ^MINGW.*  ]]; then
    # https://stackoverflow.com/questions/32597209
    alias python='winpty python'
    alias python3='winpty python3'
fi

if [ -x "$(command -v python3)" ]; then
    if ! [ -x "$(command -v python)" ]; then
        alias python='python3'
    fi
fi
alias py='python'
alias py3='python3'
alias p='python'
alias p3='python3'
alias wp='which python'
alias wp3='which python3'
alias i='ipython'
alias j='jupyter notebook'

alias cm='cd ~/catkin && catkin_make'

#alias cs='cd $MT_SRC_DIR'
#alias ca='cd $MT_ANALYSIS_DIR'
#alias co='cd $MT_OUTPUT_DIR'
#alias ci='cd $MT_INPUT_DIR'
#alias cr='cd $MT_PLAYBACK_DIR'
#alias trajecgui='trajectory_viewer_gui_v2.py'

# if this ever causes problems with logs, can also include ROS_LOG_DIR=/home/user/.ros/log
alias roslaunch='ROS_HOME=`pwd` roslaunch'
# TODO why does tab completion not seem to work with this one? can i make it?
alias rl='ROS_HOME=`pwd` roslaunch'

# TODO had alias e=exit on blackbox, and c=clear, and i might prefer those
#alias e='cd $EXP_DIR'
#alias cdc='cd ~/src/al_imaging'
# d=data
#alias cdd='cd /media/threeA/Tom/flies/'
# & cdd that uses env var to go to data

alias fd='roscd'
# TODO idk why this worked in one terminal, but now seems to have issues in new ones...
# see where .bash_aliases is sourced in whatever sources completion stuff i suppose
_completion_loader roscd
# got from `complete -p roscd`
complete -o nospace -F _roscomplete_sub_dir fd

alias mtdir='rosrun multi_tracker mk_date_dir.py'
# TODO maybe have expdir make a directory for whicever acquisition pipeline i'm using at the moment?
alias expdir='rosrun multi_tracker mk_date_dir.py'

## could quote roslaunch in those for which i dont want to use above roslaunch alias
## or use other methods of escaping like 1 (2?) backslashes preceding
## p for Play
#alias p='roslaunch multi_tracker play_delta_video.launch'
## t for Tracking
#alias t='roslaunch multi_tracker tracking.launch'
## f for roi_Finder.py
#alias f='roslaunch multi_tracker detect_roi_tracking.launch'
## u for Usb_cam
#alias u='roslaunch multi_tracker rectified_usb_cam.launch'
## d for Directory
## TODO how to cd to it? have it return directory name?
#alias d='expdir'
## c for Camera
## should i use any unrectified cameras?
## TODO make this
##alias c='roslaunch multi_tracker pointgrey_usb.launch'

alias arduino='~/arduino-1.8.0/arduino'

# Library Update
# TODO maybe find whichever arduino is actually installed?
alias lu='cd $HOME/arduino-1.8.3/libraries rm -rf ros_lib && rosrun rosserial_arduino make_libraries.py .'

# TODO central config file for specifying target of e or a? in .ros maybe (has to not get cleared)?
# should it just be a launch file?
# TODO way to launch most recent pair of camera and tracking pipeline?
#alias a='u & f'

# TODO refactor / rename this. maybe break mt_aliases into another file so they can be installed
# and this can be generated from them?
alias mt_aliases='printf "e - cd to experiment directory (set with \$EXP_DIR)\np - play back delta video\nt - standard tracking pipeline\nf - detect rois and launch a tracking pipeline for each\nu - launch rectified usb_cam node (camera must have been calibrated)\nd - makes a directory named as the date, and populates it with template configuration files\nc - launch unrectified pointgrey camera\n"'

# might need sudo on pgrep?
# sudo xargs or xargs sudo?
alias labpython='pgrep python -u lab | sudo xargs kill'
alias tompython='(pgrep python -u tom | sudo xargs kill); (pgrep record -u tom | sudo xargs kill)'

# TODO could use same set of env vars i was planning on using for analysis
# TODO how-to on setting up this kind of key -> lab github
alias transfer_data='rsync -avPurz -e "ssh -vi $HOME/.ssh/for_rsync_to_analysis" $HOME/data tom@gerty:/home/tom/data'
alias transfer_data2='rsync -avPurz -e "ssh -vi $HOME/.ssh/for_rsync_to_analysis" $HOME/data tom@cthulhu:/home/tom/data'
# TODO why does the syntax seem to be different w/o -e? (same paths put data
# inside data)
alias transfer_data3='rsync -avPurz $HOME/data tom@atlas:/home/tom/'

# TODO provide option to specify a subdirectory / automate this whole process
#alias gather_tracking='rsync -avPurz $HOME'

alias v='vi'

# It seems if it was saved w/ a diff version of python or something, nothing is
# printed? kind of odd, considering it worked with 3 and I thought i would have
# saved the temporary mappings I tested it on in 2 (ROS). Syntax for Python 2?
alias pickle='python -mpickle'
alias pickle3='python3 -mpickle'

alias ssid="nmcli -t -f active,ssid dev wifi | egrep '^yes' | cut -d\' -f2"

# TODO detect venv? other tools to accomplish this?
# TODO add pyvenv stuff (+ 16.04 install (DigitalOcean)?) to cheatsheet
# TODO make "env" if not there, then source?
# TODO need to fix pythonpath to avoid problems from ROS additions (or other)?
alias a='. env/bin/activate'

# Starting vim in insert mode with paste option set.
alias vip="vim +startinsert -c 'set paste'"

# For faster debugging
# -cc enters the first continue command, so script starts without delay.
alias dpy="python -mipdb -cc"
alias dpy2="python2 -mipdb -cc"
alias dpy3="python3 -mipdb -cc"

alias nt="cd /mnt/nas/Tom"
alias mb="cd /mnt/nas/mb_team"
alias mbr="cd /mnt/nas/mb_team/raw_data"
alias mba="cd /mnt/nas/mb_team/analysis_output"
# TODO make aliases like mbr/mba above, but to go to latest fly dir / first of
# latest unanalyzed (mbrl/mbal)

alias ..="cd .."
alias src="cd ~/src"
alias sr="cd ~/src"
alias cs="cd ~/src"
alias d="cd ~/src/dotfiles && ls"
alias s="cd ~/src/scripts && ls"
alias 2p="cd ~/src/python_2p_analysis && ls"
alias 2pp="cd ~/src/python_2p_analysis && vi populate_db.py"
alias 2pg="cd ~/src/python_2p_analysis && vi gui.py"
alias 2pu="cd ~/src/python_2p_analysis/hong2p && vi util.py"
alias 2pk="cd ~/src/python_2p_analysis/hong2p && vi kc_mix_analysis.py"
alias 2ps="cd ~/src/python_2p_analysis/scripts && ls"
alias cu="cd ~/src/chemutils && ls"
alias no="cd ~/src/natural_odors && ls"

# Uses a script in my scripts repo.
alias gitgit="git remote -v | change_git_auth.py g | xargs git remote set-url origin"
alias githttps="git remote -v | change_git_auth.py h | xargs git remote set-url origin"
alias gitssh="git remote -v | change_git_auth.py s | xargs git remote set-url origin"


# TODO add a version of this that only searches files tracked by git
# (to automatically avoid any build artifacts, like python egg stuff, etc)
# Main difference between -r and -R seems to be that -R follows symlinks.
# Using this syntax for multiple exclude-dir b/c it's friendly with grepym.
alias grepy="grep -R --exclude-dir=.direnv --exclude-dir=site-packages --include=\*.py"
# This will lookup and use the alias definition above at runtime.
alias grepym="grep_py_in_my_repos.py"

# TODO TODO alias to cd to a folder and then vi any .py files 1) w/ prefix of
# foldername or 2) lone .py files

alias dlwebsite="dlwebsite.py"

alias lr='ls -ltr'
alias bashrc="vi ~/.bashrc"
alias brc="vi ~/.bashrc"
alias sb="echo 'reloading ~/.bashrc'; source ~/.bashrc"
alias bashaliases="vi ~/.bash_aliases"
alias ba="vi ~/.bash_aliases"
alias ba="vi ~/.bash_aliases"

# [d]isable [c]ustom [e]xcepthook
alias dce="PYMISTAKE_DISABLE=1"

# First part will deactivate conda if it is active, failing silenting if no
# conda. This is because conda can prevent gsettings from being saved, see err:
# "...Using the 'memory' GSettings backend.  Your settings will not be saved or
# shared with other applications."
# TODO close ALL open envs (I think activating a non-base and then something
# else will cause deactivate to essentially pop one off the stack)
# TODO reactivate any closed conda envs after gsettings
# (or since that might impose some lag, at least warn that conda was closed
# IF it was)
# TODO TODO this might not be deactivating correctly...
# (which python changes, but lots of CONDA env vars are still there, and PS1
# does not change. maybe make it also update PS1? do something other than this
# redirect?)
alias gsettings="conda deactivate &> /dev/null; gsettings"

# Pass a PID
alias plin="pstree -l -s -p"

# https://www.cyberciti.biz/faq/how-to-find-my-public-ip-address-from-command-line-on-a-linux/
alias pubip="dig +short myip.opendns.com @resolver1.opendns.com"

