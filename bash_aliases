
#alias plots='scp tom@eftm.duckdns.org:~/lab/hong/src/*html .'
alias fiji='$HOME/Fiji.app/ImageJ-linux64'

#alias atty='stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts -hupcl'

alias x='xdg-open'
alias rs='rsync -auvP'
alias cpconf='cp $HOME/catkin/src/multi_tracker/examples/sample_data/config_20160412_134708_N1.py .; mv config_20160412_134708_N1.py config_`ls -p | grep bag | sed s/_delta_video.bag//g`.py'
alias py='python'
alias i='ipython'
alias j='jupyter notebook'

alias cm='cd ~/catkin && catkin_make'

alias cs='cd $MT_SRC_DIR'
alias ca='cd $MT_ANALYSIS_DIR'
alias co='cd $MT_OUTPUT_DIR'
alias ci='cd $MT_INPUT_DIR'
alias cr='cd $MT_PLAYBACK_DIR'
alias trajecgui='trajectory_viewer_gui_v2.py'

# if this ever causes problems with logs, can also include ROS_LOG_DIR=/home/user/.ros/log
alias roslaunch='ROS_HOME=`pwd` roslaunch'
# TODO why does tab completion not seem to work with this one? can i make it?
alias rl='ROS_HOME=`pwd` roslaunch'

alias e='cd $EXP_DIR'
alias cdc='cd ~/src/al_imaging'
# & cdd that uses env var to go to data

alias mtdir='rosrun multi_tracker mk_date_dir.py'
# TODO maybe have expdir make a directory for whicever acquisition pipeline i'm using at the moment?
alias expdir='rosrun multi_tracker mk_date_dir.py'

# could quote roslaunch in those for which i dont want to use above roslaunch alias
# or use other methods of escaping like 1 (2?) backslashes preceding
# p for Play
alias p='roslaunch multi_tracker play_delta_video.launch'
# t for Tracking
alias t='roslaunch multi_tracker tracking.launch'
# f for roi_Finder.py
alias f='roslaunch multi_tracker detect_roi_tracking.launch'
# u for Usb_cam
alias u='roslaunch multi_tracker rectified_usb_cam.launch'
# d for Directory
# TODO how to cd to it? have it return directory name?
alias d='expdir'
# c for Camera
# should i use any unrectified cameras?
# TODO make this
alias c='roslaunch multi_tracker pointgrey_usb.launch'

# Library Update
# TODO maybe find whichever arduino is actually installed?
alias lu='cd $HOME/arduino-1.8.3/libraries rm -rf ros_lib && rosrun rosserial_arduino make_libraries.py .'

# TODO central config file for specifying target of e or a? in .ros maybe (has to not get cleared)?
# should it just be a launch file?
# TODO way to launch most recent pair of camera and tracking pipeline?
alias a='u & f'

# TODO refactor / rename this. maybe break mt_aliases into another file so they can be installed
# and this can be generated from them?
alias mt_aliases='printf "e - cd to experiment directory (set with \$EXP_DIR)\np - play back delta video\nt - standard tracking pipeline\nf - detect rois and launch a tracking pipeline for each\nu - launch rectified usb_cam node (camera must have been calibrated)\nd - makes a directory named as the date, and populates it with template configuration files\nc - launch unrectified pointgrey camera\n"'

# might need sudo on pgrep?
# sudo xargs or xargs sudo?
alias labpython='pgrep python -u lab | sudo xargs kill'
alias tompython='(pgrep python -u tom | sudo xargs kill); (pgrep record -u tom | sudo xargs kill)'
