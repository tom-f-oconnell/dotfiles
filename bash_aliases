
# TODO TODO TODO make sure all reused names are at least given local scope

# TODO TODO maybe modify how all aliases (+ maybe fns?) are defined in here, so
# that warnings get printed (when ~/.bashrc (and thus this) is sourced) in cases
# where one of my definitions shadows something, so that i can change the name

# TODO maybe something to print a reminder for any aliased command?
# like (per alias) once within a session maybe? or once per reboot?

# TODO TODO some command to search the targets of all aliases (+ fns?) for some
# pattern (like 'git'), to print the aliases (+ fns?) (i.e. that use that
# command)
# (delete other comment about this if i can find it / if it still exists
# somewhere below)

# TODO try to improve git diff autocomplete (so it only completes files
# that are different. i.e. those that would show up in red in git status)

# TODO store state of the path of the cwd of the most recent active terminal,
# so new terminals can cd right there with some shortcut

# Takes one argument, a string to start the prompt with.
# TODO option to require enter press?
function confirm() {
    # https://stackoverflow.com/questions/1885525
    read -p "$1? (y/n) " -n 1 -r
    echo # (optional) move to a new line
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # 0 is True in BASH
        return 0
    else
        return 1
    fi
}

# https://stackoverflow.com/questions/4023830
function verlte() {
    # I just split up his original command like this because it doesn't screw up
    # my syntax highlighting in the rest of the file.
    local rhs="`echo -e "$1\n$2" | sort -V | head -n1`"
    [ "$1" = "$rhs" ]
}
function verlt() {
    [ "$1" = "$2" ] && return 1 || verlte $1 $2
}
#verlte 2.5.7 2.5.6 && echo "yes" || echo "no" # no
#verlt 2.4.10 2.4.9 && echo "yes" || echo "no" # no
#verlt 2.4.8 2.4.10 && echo "yes" || echo "no" # yes
#verlte 2.5.6 2.5.6 && echo "yes" || echo "no" # yes
#verlt 2.5.6 2.5.6 && echo "yes" || echo "no" # no

# opens a file in Fiji, and appends current directory before argument so Fiji
# doesn't freak out
function fp() {
    $HOME/Fiji.app/ImageJ-linux64 $(pwd)/$1
}

function git_commit_add() {
    # TODO delete after debugging
    echo "CHECK THAT LOGGED COMMIT MESSAGE IS ACTUALLY WHAT YOU WANT"
    #

    # TODO if possible, find some way to see if raw command that was entered to
    # trigger this fn contains characters that bash would need escaped to have
    # them preserved in the commit message (" and ' for instance, it seems),
    # and warn / fail if we have them, or use the raw command to insert them
    # escaped as appropriate
    # TODO or maybe just assert that the # of args passed in is 1, forcing the
    # message to be specified w/ some kind of quotes already around it?

    # I wasn't able to find cases where any of these actually made a difference.
    #local args="$(echo "$@" | sed -e 's/"/\"/g')"
    #echo $@
    #echo "\$@: $@"
    #echo "args: $args"
    git commit -am "$@"
}

# for aliases where the arguments should go in the middle
function ltx() {
    FILE=$1
    PREFIX=${FILE%%.*}
    latex $1
    RET=$?
    rm "$PREFIX.log"; rm "$PREFIX.aux"
    if [ $RET -eq 0 ]; then
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

# Accepts one argument, a string to be eval'd as a command.
# Echos both stdout and stderr from original command to stdout.
# Exists because out="$(<cmd> 2>&1)" does not capture stderr.
# https://stackoverflow.com/questions/13806626
# WARNING: not sure this works too well...
#function stderr_too() {
#    echo "CMD: \"$1\""
#    TMP=$(mktemp)
#    echo "TMP: $TMP"
#    var=$(eval $1 > "$TMP" 2>&1)
#    eval_exit_code="$?"
#    cat "$TMP"
#    rm "$TMP"
#    return $eval_exit_code
#}


# Use like:
# ```
# if _check_one_nonexistant_arg "$@"; then
#     <what to do if validation was successful>
# fi
# ```
#
# ...or:
# ```
# if ! _check_one_nonexistant_arg "$@"; then
#     return 1
# fi
# ```
#
# Above syntax options work at least in the called-from-within-a-function case.
function _check_one_nonexistant_arg() {
    if [ "$#" -ne 1 ]; then
        # TODO TODO TODO fix so it actually reports function name if possible
        # currently it reports 'bash' (as $0). probably same below.
        #echo "Usage: $0 FILE_TO_MAKE" >&2
        echo "must pass single non-existant filename"
        return 1
    elif [ -e "$1" ]; then
        echo "'$1' already exists"
        return 1
    fi
    return 0
}

# See comments above previous function `_check_one_nonexistant_arg` for appropriate
# syntax when using this.
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

# This is intended to return 0 (for True, because BASH) if the function is
# called from within EITHER a normal virtual environment OR a conda environment.
function in_virtual_env() {
    # From combining two parts of Victoria Stuart's answer here:
    # https://stackoverflow.com/questions/1871549
    # to also support conda environments (which might all be included in first case)
    # TODO see same link for how to generalize the below line to not err in some earlier
    # versions of python
    if [ -n "$CONDA_DEFAULT_ENV" ]; then
        return 0
    elif [ -n "$VIRTUAL_ENV" ]; then
        return 0
    fi

    # Contains both commented attempt and solution I ended up using:
    # https://stackoverflow.com/questions/15454174

    # Assuming that this will always work in the case where we ARE in some
    # kind of virtual env, because AFAIK, 'python' is always pointing to the
    # venv python in any kind of venv (including conda ones).
    # Will just let this fail if no python available.

    # This came from a comment by BrendanSimon on kjo's answer.
    # May have some edge cases, as the comment didn't get much scrutiny...
    # This at least seems to work correctly (w/ error check after) for the
    # python3.6 venv and conda base env on blackbox (May 2020).
    IN_VENV=$( python -c 'import sys ; print( 0 if sys.prefix != sys.base_prefix else 1 )' 2>/dev/null )
    if ! [ "$?" -eq 0 ]; then
        # Presumably the python check erring means we are NOT in a venv...
        # 1 for False here (b/c BASH)
        return 1
    fi
    return $IN_VENV

    # didn't work w/ python3.6 venv created w: python -m venv venv
    #IN_VENV=$(python -c 'import sys; print ("1" if hasattr(sys, "real_prefix") else "0")')

    # didn't seem to be set by conda
    #if [[ "$VIRTUAL_ENV" != "" ]]; then
}

# TODO maybe i want to change this to ".venv" or (if one exists)
# a widely accepted convention here
export DEFAULT_VENV_NAME="venv"

# This SO post explains using parens rather than curly braces to enclose
# function body, so function will be evaluated in subshell, and I can use
# shopt to only change opions for the subshell.
# https://stackoverflow.com/questions/12179633
# TODO may need to rename this if this ends up shadowing some executable i use
# (maybe better to just do that now, if i'm only going to use through alias anyway...)
# TODO what was the case i found regarding conda where this was not idempotent
# (it didn't seem to refuse activating when some [conda?] thing was already active, i
# think. maybe it was on atlas 18.04 reinstall where i encountered it.)
function activate() {
    # If a virtual env is already active, just notify and exit.
    # Originally I was thinking of not notifying, but I don't want this to
    # create hard-to-debug situations down the line.
    if in_virtual_env; then
        echo "Some virtual environment already active"
        return 3
    fi

    # TODO TODO also support conda envs in single-positional-argument-passed case
    # (maybe following any existant directories of same name [but warning about
    # collision] if both are found)
    # (though we do have the alias 'ca' too currently...)

    out=$(
    shopt -s dotglob
    shopt -s nullglob

    if [ -z "$1" ]; then
        possible=(*/bin/activate */Scripts/activate)
    else
        # TODO fail w/ error message if $1 is not directory
        possible=($1/*/bin/activate $1/*/Scripts/activate)
    fi

    if [ "${#possible[@]}" -eq 0 ]; then
        # This nowarn option is so far only set when `activate` is called from
        # my `pyp` function.
        if [ -z "$nowarn" ]; then
            # TODO maybe also print paths that were searched here / globs
            echo "No virtual env activation scripts found."
        fi
        return 1
    elif [ "${#possible[@]}" -eq 1 ]; then
        echo "${possible[0]}"
        return 0
    else
        echo "Too many (${#possible[@]}) virtual env activation scripts found!"
        printf '%s\n' "${possible[@]}"
        return 2
    fi
    )
    subshell_exit_code=$?
    #echo "subshell_exit_code=$subshell_exit_code"
    # This return code means we can expect the output to be a filename.
    if [ "$subshell_exit_code" -eq 0 ]; then
        echo "Sourcing activation script found at: ${out}"
        source "$out"
        return 0
    else
        if [ -n "$out" ]; then
            printf '%s\n' "$out"
        fi

        # TODO maybe pass extra args through to activate function, so they can
        # be passed to the venv creation in this case?
        if [ "$subshell_exit_code" -eq 1 ]; then
            # Because otherwise some kind of operations are still carried out,
            # but I'm not sure whether it will effectively clear an environment
            # that has already had stuff installed into it (or not).
            if [ -d "$DEFAULT_VENV_NAME" ]; then
                printf "Would have made virtual env at $DEFAULT_VENV_NAME, "
                printf "but it already exists!\n"
                return 4
            fi
            # TODO make allow an env var to force one way or the other, without
            # prompting
            # TODO TODO just print after we decide on other args, and print full
            # command then... (or just don't print?)
            printf "Making virtual env with 'python3 -m venv "
            printf "$DEFAULT_VENV_NAME'\n"

            # TODO maybe test for 18.04 or similarly broken systems, maybe some
            # settings I broke myself through using my 18.04 setup?
            # TODO maybe test if python3 -m pip works first or something,
            # otherwise do bootstrapping?
            # TODO or maybe check pip version in venv and upgrade if it's too
            # low? (prompt to enable this behavior?)

            # NOTE: I'm not sure if there are some cases where leads to an
            # environment without a usable pip, BUT this was necessary to make a
            # virtual environment in 18.04 that used my user python3 pip, which
            # I had just upgraded via "pip install --upgrade pip" (outside of a
            # venv). Otherwise, with or without --system-site-packages, the
            # venv used the older pip that was installed before (9.0.1).
            # TODO find solution for NOT --system-site-package case. turns out
            # this only works there... (low priority cause just always upgrading
            # pip now)
            #local extra_venv_flags="--without-pip"
            local extra_venv_flags=""

            # TODO TODO accept (-n / -y) flags to answer this in advance?
            # some env var? another alias? ([an / ay] or maybe sa for w/
            # system [as is some system thing]?)?
            if confirm "Make venv with --system-site-packages?"; then
                #extra_venv_flags="${extra_venv_flags} --system-site-packages"
                extra_venv_flags="--system-site-packages"
            fi

            # TODO maybe refactor so same command used to print is eval-ed
            # to create the venv...

            # python3 because this isn't supported with python 2, which might
            # be what "python" refers to in some places.
            python3 -m venv $DEFAULT_VENV_NAME $extra_venv_flags
            venv_exit_code=$?
            if ! [ "$venv_exit_code" -eq 0 ]; then
                printf "Creating virtual env failed with exit code: "
                printf "$venv_exit_code\n"
            else
                # TODO could maybe refactor so searching code is its own fn
                # (probably use parens rather than curly for fn body so shopt
                # still works), so that i can call it w/ a lower number of paths
                # (we know which parent dir, just not whether 'bin' or 'Scripts'
                # in the second level here...)
                # (doesn't matter much though)

                # Assuming this will succeed (and will not recurse further)
                # (it should succeed, given we just made it...)
                activate

                # TODO maybe just print last line of this output?
                # Mostly for the case were we are using one of the Ubuntu system
                # pythons, with their perpetually out of date pips.
                pip install --upgrade pip

                # TODO TODO maybe prompt to install ipdb? or just do it?

                #local venv_pip_ver=`pip --version | awk '{print $2}'`
                #echo $venv_pip_ver
                ## Some what arbitrary cutoff between 9.0.1 (stock 18.04 system
                ## python3 pip) and latest pypi version (as of 2020-08-13) of
                ## 20.2.2
                #if verlt $venv_pip_ver 18.0.0; then
                #    pip install --upgrade pip
                #fi
            fi
        fi

        # TODO this work to return diff numbers, or is this treated like a
        # string, and is that bad here?
        # TODO test that the 1 and 2 are preserved, and check they are treated
        # just like the 0 in the "return 0" above!
        return $subshell_exit_code
    fi
}

# It's important this is not named "deactivate", as basically all venv tools
# will probably shadow anything with that name when they activate an
# environment.
function my_deactivate() {
    # This just seems to be the name of the active environment.
    # https://stackoverflow.com/questions/51266880
    if [ -n "$CONDA_DEFAULT_ENV" ]; then
        conda deactivate
        return 0
    fi

    local deactivate_type="$(type -t deactivate)"

    if ! [ "$?" -eq 0 ]; then
        echo "No virtual environment active"
        return 0
    fi

    # This is the type I got for deactivate when in a venv created like:
    # python3 -m venv venv  (w/ Python 3.6)
    if [ "$deactivate_type" == "function" ]; then
        # TODO if there are some conda versions where this will also be a fn,
        # maybe test for them (having an active env in current shell) some other
        # way
        deactivate
    else
        >&2 echo "In my_deactivate, unexpected deactivate type: $deactivate_type"
        return 1
    fi
}

# Using this to overload the 'd' alias, to have more functionality on my one
# character aliases. We can always tell it's intended to be 'deactivate', if
# there are no arguments.
function diff_or_deactivate() {
    if [ "$#" -eq 0 ]; then
        my_deactivate
    else
        diff "$@"
    fi
}

# `hub` also uses this
export GITHUB_USER="tom-f-oconnell"
# Just setting this for possible `hub` benefit now, though it still requires
# password on first use...
export HUB_PROTOCOL="ssh"

# `clone` (below) will use earlier entries in this array first.
declare -a MY_GITHUB_ACCOUNTS=($GITHUB_USER "ejhonglab")
export MY_GITHUB_ACCOUNTS

# TODO TODO try to get tab completion (on top of any existing, if there, and if
# modifying it like this is possible) for hub organization (from
# MY_GITHUB_ACCOUNTS, if not directly from github.com) and for repos within them
# TODO TODO or at least make an alias to make repos w/ current dir name w/
# different github user / org (maybe just one alias to do this for ejhonglab)
# TODO something like hub create ejhonglab/"$(basename `pwd`)"

# TODO make fn gitproj/ghproj/gproj/gpr or something to do this, but without
# expecting it to be a python project
function pyp() {
    if _check_one_nonexistant_dir_arg "$@"; then
        # TODO if hub exists, maybe ensure no (non-empty) project of same name
        # already on your github here, before anything else happens
        # (and fail or warn / prompt to clone?)

        mkdir $1
        cd $1

        # TODO maybe replace w/ a test that would also catch aliases?
        # `type` / some custom thing
        if [ -x "$(command -v python3)" ]; then
            # TODO generalize to also test if the corresponding layout
            # defininition for the line below exists in ~/.direnvrc (maybe check
            # through direnv though, rather than checking line)
            # TODO maybe also / instead, check if hook is active
            try_direnv=1
            if [ "$try_direnv" -eq 0 ] && [ -x "$(command -v direnv)" ]; then
                if confirm "Make .envrc to start venv?"; then
                    echo "layout python-venv" > .envrc
                    # TODO maybe conda deactivate first (if applicable)? (why
                    # would i want to deactivate, again? conda specific of all
                    # venvs)
                    direnv allow
                    # TODO need to do anything else so it's actually activated?
                    # (like cd-ing up and back)? (just want behavior to be
                    # consistent w/ other case that makes and then activates
                    # venv)
                fi
            fi
            if confirm "Make (and activate) venv?"; then
                # TODO TODO maybe add prompt to select --system-site-packages
                # flag
                # TODO can i just set a sequence of space separate env vars
                # before command?
                nowarn="0" activate
            fi
        fi
        git init
        # Using this last part rather than $1 in case of trailing slashes
        project="$(basename `pwd`)"
        git remote add origin git@github.com:$GITHUB_USER/$project

        # TODO maybe set origin master thing (if possible before first push) so
        # don't need -u flag on first push

        # This is Github's own automation tool
        if [ -x "$(command -v hub)" ]; then
            # TODO this doesn't seem to avoid need for password on first use.
            # is there a way?
            #export HUB_PROTOCOL="ssh"

            # By default, this makes the repo with the name of the current
            # directory (which we cd'd to above).
            echo "Making public repo on your Github with 'hub'"
            hub create
        fi

        py_file="${project}.py"
        if confirm "Make (and open in vim) $py_file?"; then
            vim $py_file
        fi
    fi
}

# Whether dir is subdir / root of a git repo.
# https://stackoverflow.com/questions/2180270
function is_git_dir() (
    if ! [ -d "$1" ]; then
        #echo "first argument to is_git_dir was not a directory! returning false"
        return 1
    fi
    # since most git commands rely on being inside the git repo. this function
    # is in a subshell (parens) to avoid changing working dir of caller.
    cd "$1"
    git rev-parse --git-dir > /dev/null 2>&1
)

# TODO test!
# Does not seem to return True for something added but not committed, but that's
# OK.
function is_git_tracked() (
    if ! [ -f "$1" ]; then
        #echo "first argument to is_git_tracked was not a file! returning false"
        return 1
    fi
    # since most git commands rely on being inside the git repo. this function
    # is in a subshell (parens) to avoid changing working dir of caller.
    cd $(dirname $1)
    # https://stackoverflow.com/questions/2405305
    git ls-files --error-unmatch "$1" > /dev/null 2>&1
)

# TODO TODO maybe optionally accept a line of files, and then use those instead
# of all files?
function link_to_vagrant() {
    local vagrant_dirs_root=~/src/misc
    if ! [ -d "$vagrant_dirs_root" ]; then
        echo "$vagrant_dirs_root did not exist. clone my misc repo there."
        return 1
    fi

    local lrd="$(lsb_release -d)"
    local vagrant_dir
    local logfile_prefix
    if echo $lrd | grep "18.04" -q; then
        vagrant_dir="${vagrant_dirs_root}/18.04_vagrant"
        logfile_prefix="ubuntu-bionic-18.04"
    elif echo $lrd | grep "16.04" -q; then
        vagrant_dir="${vagrant_dirs_root}/16.04_vagrant"
        # TODO check this one is right
        logfile_prefix="ubuntu-xenial-16.04"
    else
        echo "lsb_release -d did not have either 16.04 or 18.04"
        return 2
    fi

    if ! [ -d "$vagrant_dir" ]; then
        echo "vagrant directory $vagrant_dir did not exist"
        return 3
    fi

    if ! [ -z "${just_cleanup}" ]; then
        printf "destroying vagrant machine (if exists)... "
        vagrant destroy -f > /dev/null 2>&1
        echo "done"

        #if is_git_tracked Vagrantfile; then
        #    echo "Not cleaning up because Vagrantfile tracked by git"
        #    return 1
        #fi

        # Only removing Vagrantfile if it is a symbolic link.
        if [ -L Vagrantfile ]; then
            rm -fv Vagrantfile
        fi
        rm -rfv .vagrant/
        # ex: ubuntu-bionic-18.04-cloudimg-console.log
        rm -fv ${logfile_prefix}-cloudimg-console.log

        return
    fi

    if confirm "copy Vagrantfile instead of linking?"; then
        cp -v $vagrant_dir/Vagrantfile .
    else
        ln -sv $vagrant_dir/Vagrantfile .
    fi

    if confirm "check for box update?"; then
        vagrant box update
    fi

    if confirm "start vagrant and ssh in?"; then
        # This directory seems to contain only nested empty directories
        # when my 18.04 virtualbox VM is down.
        # -quit is to stop at first.
        # TODO is this test command outputting a 0/1? or is my confirm above?
        # i keep seeing it...
        # TODO what is the point of this find command again?
        # fix in copy case (or really, case where vagrant has not yet been run
        # from $vagrant_dir, by using symlinked version, i think / has been
        # destroyed and not run since). outputs this:
        # find: ‘/home/tom/src/misc/18.04_vagrant/.vagrant/’: No such file or
        # directory.
        # 0
        if find ${vagrant_dir}/.vagrant/ -type f -print -quit | wc -l ; then
            # TODO TODO possible to pause for `confirm` y/n prompt to ask
            # whether to `vagrant box update` if the up command (that the one
            # that prints it?) has that out of date warning? e.g.
            # ==> default: A newer version of the box 'ubuntu/bionic64' for
            # provider 'virtualbox' is
            # ==> default: available! You currently have version '20200605.0.0'.
            # The latest is version
            # ==> default: '20200819.0.0'. Run `vagrant box update` to update.
            vagrant up
        fi
        vagrant ssh
    fi

    # seems like the below style of approaches won't work, or will be hard to
    # get to work

    #if ! [ -z "${just_cleanup}" ]; then
    #    # TODO maybe at least warn if args were passed...
    #    echo "deleting *all* symbolic links under $vagrant_dir and exiting"
    #    find $vagrant_dir -type l -delete
    #    return
    #fi

    ##echo "linking to current directory under $vagrant_dir"
    ##ln -rs . ${vagrant_dir}/$(basename `pwd`)

    ## TODO is symlinking everything like this going to present problems?

    #if [ -z "$1" ]; then
    #    # The -r (relative) flag seems necessary for this to work as written.
    #    echo "linking EVERYTHING in current directory under $vagrant_dir"
    #    ln -rs * ${vagrant_dir}/.
    #else
    #    echo "linking files from arguments to $vagrant_dir:"
    #    for f in "$@"
    #    do
    #        echo "$f"
    #        # TODO check $f? abs and don't use -r?
    #        ln -rs $f ${vagrant_dir}/.
    #    done
    #fi

    ## TODO maybe either edit vagrant config or something else so that it cd's
    ## into the symlinked dir upon up/ssh? (just symlinking everything now...)

    #if confirm "cd to vagrant directory and ssh in?"; then
    #    cd ${vagrant_dir}
    #    # This directory seems to contain only nested empty directories
    #    # when my 18.04 virtualbox VM is down.
    #    # -quit is to stop at first.
    #    if find ${vagrant_dir}/.vagrant/ -type f -print -quit | wc -l ; then
    #        vagrant up
    #    fi
    #    vagrant ssh
    #fi
}

alias cr='cp -r'
alias cpr='cp -r'

# TODO maybe 'git status' if it's a git repo (factor to fn)?
alias c='cd'
# Uses https://github.com/cykerway/complete-alias installed via my dotfiles setup
complete -F _complete_alias c

function c1() {
    cd "../$1"
}
function c2() {
    cd "../../$1"
}
function c3() {
    cd "../../../$1"
}
# https://stackoverflow.com/questions/38737675
# TODO TODO replace this w/ _dir_completion below
function _cd_up_n() {
    # https://www.gnu.org/software/bash/manual/html_node/Programmable-Completion-Builtins.html
    # "$1 is the name of the command whose arguments are being completed, $2 is the word
    # being completed, and $3 is the word preceding the word being completed"
    local cmd=$1 cur=$2 pre=$3
    local _cur compreply

    local completion_dir
    if [ "$cmd" = "c1" ]; then
        completion_dir=".."
    elif [ "$cmd" = "c2" ]; then
        completion_dir="../.."
    elif [ "$cmd" = "c3" ]; then
        completion_dir="../../.."
    else
        >&2 echo "command $cmd not supported by completion fn _cd_up_n"
        return 1
    fi

    _cur="$completion_dir/$cur"
    compreply=( $( compgen -d "$_cur" ) )
    COMPREPLY=( ${compreply[@]#$completion_dir/} )
    if [[ ${#COMPREPLY[@]} -eq 1 ]]; then
        COMPREPLY[0]=${COMPREPLY[0]}
    fi
}
complete -F _cd_up_n -o nospace -S '/' c1
complete -F _cd_up_n -o nospace -S '/' c2
complete -F _cd_up_n -o nospace -S '/' c3

# Usage: complete -C "_dir_completion <path-to-complete-from>" <func-to-complete-for>
#
# $1 should be a path that we want subdirectory completions for.
#
# https://stackoverflow.com/questions/56178162
function _dir_completion() {
    # https://www.gnu.org/software/bash/manual/html_node/Programmable-Completion-Builtins.html
    # "$1 is the name of the command whose arguments are being completed, $2 is the word
    # being completed, and $3 is the word preceding the word being completed"
    #
    # (and since the argument we pass via -C takes $1, it shifts all these over one)
    local cmd=$2 cur=$3 pre=$4
    local _cur compreply

    local completion_dir=$1

    _cur="$completion_dir/$cur"
    compreply=( $( compgen -d "$_cur" ) )
    COMPREPLY=( ${compreply[@]#$completion_dir/} )
    if [[ ${#COMPREPLY[@]} -eq 1 ]]; then
        COMPREPLY[0]=${COMPREPLY[0]}
    fi

    # Since -C expects completions printed this way, rather than setting COMPREPLY
    for reply in "${COMPREPLY[@]}"
    do
        printf '%s\n' "$reply"
    done
}

# Usage:
# complete -C "_dir_completion_via_dir_fn <path-to-complete-from>" <func-to-complete-for>
#
# $1 should be a command that prints a path that we want subdirectory completions for.
#
# Use over _dir_completion if you need the path to be computed when the completion is
# requested, rather than when this file is sourced.
function _dir_completion_via_dir_fn() {
    if [ -x "$(command -v $1)" ]; then
        local completion_dir
        if completion_dir=$( $1 2>/dev/null ); then
            _dir_completion ${completion_dir} ${@:2}
        fi
    fi
}

# TODO maybe add these:
# ti (test import) ~ python -c 'import $1'
# ppv (pv is common i think) (python package version)
#   ~ python -c 'import $1; print($1.__version__)'
#     (check pip first though? or both?)

alias pf="pip freeze"
alias pfg="pip freeze | grep"

# TODO TODO add some kind of requirements.txt file to my dotfiles with packages to
# always install fresh in all venvs created through this function. include things like:
# - ipdb
# - darglint / black / etc (unless there is some other way to make the system versions
#   available despite not created an env with --system-site-packages. i forget, did pip
#   installing black for system python3 in 18.04 really make it work in some/all other
#   python3 venvs? or was it another black being used?)
# TODO TODO + add option (probably would want an alias with one additional letter, like
# 'ab' ([b]are) to map to it, to not always require another prompt at startup.)
alias a='activate'
alias d="diff_or_deactivate"


if ! [ -x "$(command -v mamba)" ]; then
    export MAMBA_OR_CONDA="conda"
else
    export MAMBA_OR_CONDA="mamba"
fi

alias ca='conda activate'
# Perhaps also see: https://github.com/tartansandal/conda-bash-completion
# (only if useful for other commands. for `conda activate`, this works.)
# Requires my scripts repo to be on path.
complete -o nosort -C list_conda_envs.py ca

#complete -F _complete_alias ca

# [c]onda d[e]activate
alias ce='conda deactivate'
# [c]onda [de]activate
alias cde='conda deactivate'

alias crm="conda env remove --name"
complete -o nosort -C list_conda_envs.py crm

function is_yaml() {
    if [[ ( "$#" != 1 ) ]]; then
        >&2 echo "is_yaml: must pass single filename input"
        return 2
    fi
    # NOTE: not checking we don't have other arguments and not parsing to check it is
    # valid yaml
    if [[ "$1" == *.yml ]] || [[ "$1" == *.yaml ]]; then
        if [ -f "$1" ]; then
            return 0
        else
            >&2 echo "is_yaml: $1 did not exist"
            return 3
        fi
    fi
    return 1
}
function single_yaml_arg() {
    local retcode
    local yaml_arg=""
    for x in "$@"
    do
        is_yaml "$x"
        retcode=$?
        case $retcode in
            # 0=true
            0)
                if [ -n "$yaml_arg" ]; then
                    >&2 echo "more than one YAML argument"
                    return 4
                fi;;

            # 1=false
            1) continue;;

            # anything other than 0 or 1 = error
            *) return $retcode;;
        esac

        if [ $retcode -eq 0 ]; then
            yaml_arg="$x"
        fi
    done
    if [ -n "$yaml_arg" ]; then
        echo "$yaml_arg"
    fi
}

function conda_create_from_yaml() {

    # TODO come up w/ mamba replacement for this command (that also uses mamba to
    # download), referencing https://github.com/mamba-org/mamba/issues/633 (even as-is,
    # still faster using mamba, just not as fast as it could be. mamba does solving but
    # not downloading. solving more important anyway.)

    # Excluding the -f flag so we can add it later in the appropriate place
    local create_cmd="${MAMBA_OR_CONDA} env create"

    local yaml_arg
    if ! yaml_arg=$(single_yaml_arg "$@"); then
        return 1
    fi

    if [ -z "$yaml_arg" ]; then
        yaml_arg="environment.yml"
        echo "YAML path not passed. Trying ./environment.yml"
        # So we get the same error message if it doesn't exist.
        if ! is_yaml "$yaml_arg"; then
            return 1
        fi
    fi

    local args_arr=( "$@" )
    if [ -n "$yaml_arg" ]; then
        args_arr=( "${args_arr[@]/$yaml_arg}")
    fi

    local create_cmd_with_args="${create_cmd} -f ${yaml_arg} ${args_arr[*]}"
    eval "${create_cmd_with_args}"
}
# [c]onda (create from) [f]ile
alias cf='conda_create_from_yaml'
# https://unix.stackexchange.com/questions/566390
# Requires extglob to be on (check via `shopt extglob`), but it seems on in (some of) my
# current systems. https://stackoverflow.com/questions/22854848
complete -f -o plusdirs -X '!*.@(yaml|yml)' cf

# TODO parse "name: <name>" line to complete activate command
#alias cfa='conda env create -f && conda activate <TODO>'
alias cls='conda env list'


alias rv="echo 'rm -rf venv' && rm -rf venv"

alias pp="pyp"

# TODO maybe add (aliases->) fn to make a venv w/ arbitrary python version in
# tmp dir?

alias vu="vagrant up && vagrant ssh"
alias vr="vagrant reload && vagrant ssh"
alias vd="vagrant destroy -f"
alias vs="vagrant ssh"
alias vb="vagrant box"
alias vbu="vagrant box update"

# TODO maybe change to va and vd to be kinda consistent w/ how my python venv
# aliases work? (from vl and vc) (would need to remove / change separate vd
# destroy alias above)

# [v]agrant [l]ink (not a real command)
alias vl="echo 'Use alias vc to cleanup Vagrant when done'; link_to_vagrant"
# [v]agrant [c]leanup (not a real command)
alias vc="just_cleanup=0 link_to_vagrant"

# TODO TODO maybe just use "docker attach" anyway...
# https://stackoverflow.com/questions/30960686
#function docker_bash() {
#    # TODO check $1 is valid (running?) image if need be
#    sudo docker exec -it "$1" /bin/bash
#}
#alias dsh="docker_bash"

# TODO TODO change this after figuring out which (if any) commands can be run
# without sudo + after just reconfiguring things to not need to run anything
# with sudo in most circumstance!
alias docker="sudo docker"

# TODO maybe [just] one alias that (re)builds AND then runs?
function docker_build() {
    docker build -t $(basename `pwd`) .
}
# TODO probably dont include "-it" and "bash" by default. two aliases?
function docker_run_bash() {
    # TODO does always specifying '--entrypoint bash' work (if my goal with this
    # command is to always start bash)? (it is necessary if entrypoint is
    # defined as something else in dockerfile, like in my olfactometer)
    #docker run -it $(basename `pwd`) bash
    docker run -it --entrypoint bash $(basename `pwd`)
}
# TODO maybe we actually want this as drmi though? (i.e. are we actually going
# to rm containers more than images?)
function docker_rmi() {
    docker rmi $(basename `pwd`)
}
alias db="docker_build"
alias dr="docker_run_bash"
# TODO maybe change to dbt for [t]est?
alias dbr="docker_build && docker_run"
alias drm="docker_rmi"
alias dp="docker system prune"
alias dls="echo 'Images:'; docker image list; echo ''; echo 'Containers:'; docker container list"

alias t="type"
alias w="which"
# (for checking aliases interactively)
alias al="alias"

# TODO TODO make some command (some tool exist for this?) that prints all
# aliases i have defined (maybe just those in this file) IF they contain
# some substring (e.g. "git"). maybe also want to expand aliases (if that
# wouldn't be default behavior, so that "recursive" aliases still get listed)

function git_fn() {
    if [ "$#" -eq 0 ]; then
        git status
    else
        git "$@"
    fi
}

# TODO TODO alias mv to some function that first tries git mv, then mv if not
# in a git repo (same with rm?) (or prompt if it detects the argument(s) of
# either are in git control?)
alias gmv='git mv'
complete -F _complete_alias gmv

alias grm='git rm'
complete -F _complete_alias grm

alias g='git_fn'

# https://unix.stackexchange.com/questions/216748
_completion_loader git
# The sed part just replaces the command name 'git' at end of the printed completion
# definition with 'g' (my alias for `git_fn`)
eval $(complete -p git | sed 's/git$/g/')

# TODO git init before both of these adds? init idempotent, or need to check?
alias ga='git add'
complete -F _complete_alias ga

alias gaa='git add --all'

#alias gca='git commit -am'
#alias gac='git commit -am'
alias gca='git_commit_add'
alias gac='git_commit_add'

alias gc='git commit -m'

# TODO TODO either here or somewhere else, which maybe gets called here,
# have wrap git push, so that if it fails b/c there are remote changes,
# prompt to automatically rebase and retry + prompt to open github
# in web browser (+improve message on diffs? or is it already OK?)
# (not really intending to actually do both, but rather just want the
# convenience of using gp for both pushing and pulling)
# (if i get this working, can add back gpr and maybe remove gu* stuff)
#alias gp='git pull && git push --follow-tags'

# Using this for just git push until I can fix the above and get confidence in it.
# TODO what was --follow-tags for again?
alias gp='git push'

# TODO make function gcp that passes all input ***WITHOUT MANGLING ANYTHING VIA SHELL
# INTERFERENCE*** (test!) to `git commit -m <here>` and then pushes as well
# (may more or less be equivalent to fixing my old commit fn)

# git [u]pdate (not a real command, just mnemonic)
# (and gp is already taken, and u is second letter of p[u]ll)
alias gu='git pull'
alias gur='git pull --rebase'

# TODO would need some way to find default branch name of remote (e.g. main or master?).
# new github CLI have anything useful here?
# [g]it [u]pdate [u]pstream
#alias guu='git fetch upstream && git rebase upstream/main'

# NOTE: not using 'gs' because the ghostscript package provides gs under the same name.
# Doesn't seem to be installed by default on 18.04.5 (from *.manifest file), but:
# tom@blackbox:~$ aptitude why ghostscript
# i   ghostscript-x Depends ghostscript (= 9.26~dfsg+0-0ubuntu0.18.04.14)
# tom@blackbox:~$ aptitude why ghostscript-x
# i   ubuntu-unity-desktop Depends ghostscript-x

# "[g]it [s]tash" ([a]dd)
alias gsa='git stash'
# "[g]it [st]ash"
alias gst='git stash'

# TODO as long as `gs`=ghostscript isn't installed, alias to gs too?
# TODO or just do it? is shadowing builtin stuff generally safe (i.e. are changes only
# in "interactive" terminals or something)?

# --date=short seems to just be the YYYY-MM-DD iso w/o time info, which is what I want.
alias gsl='git stash list --date=short'
# NOTE: this will not drop if the stash has a conflict.
# TODO may want to also echo a warning to drop after resolving conflict, assuming exit
# code reflects whether there was one. exit code was 1 in one test where there were
# conflicts, so this might work
alias gsp="git stash pop || printf \"\e[1;33m\nYou must manually 'git stash drop' after resolving conflict\n\e[0m\""
alias gsd='git stash drop'

# TODO maybe use whatever shell pointer to a text editor there is, rather than
# hardcoding 'vi'?
alias gi='vi .gitignore'

alias gl='git log'
# [g]it [l]og [h]ead (just show the first entry)
alias glh='git log -1'

# TODO have this output git diff HEAD if argument is already added (though maybe with a
# message also printed to notify that this is happening)
alias gd='git diff'
# TODO modify this completion to only apply to files changed ('git diff' completion
# doesn't do this either, at least as i have it installed)
complete -F _complete_alias gd

alias gdh='git diff HEAD'
complete -F _complete_alias gdh

alias gls='git ls-files'

alias grv='git remote -v'

alias grs='git remote set-url'
# TODO though make a function / script to just automatically switch it to my account,
# cause that's what i'm using this for... and i currently still have to type the
# git@github.com:tom-f-oconnell/ part
alias grso='git remote set-url origin'
alias grau='git remote add upstream'
alias gau='git remote add upstream'

# TODO maybe just use hub here? it have something like this?
# TODO how to deal w/ origin + upstream? just pick origin?
# (probably prompt and have then enter a number selecting which / both)
function open_repo_in_browser() {
    local remote
    if [ -z "$1" ]; then
        remote="origin"
    else
        remote="$1"
    fi
    # getting https link, no matter auth in 'git remote -v'
    local url="$(git remote -v | grep "$remote" | change_git_auth.py h)#readme"
    # TODO break out above so `git remote -v` is separate or otherwise figure out how
    # to make this fail gracefully w/ reminder about what it does if used not in a git
    # repo (as part of pipe, ext code seems to be 0 regardless of failure in first
    # step? though there is still stderr output)
    local browser="$(xdg-settings get default-web-browser)"
    echo "opening $url in $browser"
    xdg-open $url
}
alias gb='open_repo_in_browser'
alias gbu='open_repo_in_browser upstream'

# TODO add 'rg' alias to remove git repo if:
# - no uncommitted changes / untracked files
# - git push is already dne / repo is behind (perhaps also try it?)
# - there are no files in the repo that are not in the repo (e.g. ignored files that
#   don't show up as untracked in status output). maybe just prompt to delete them (one
#   prompt to delete all)?

# TODO make alias to vagrant up + vagrant ssh (and just ssh if already up)
# + one for tearing it down

# TODO TODO replace MY_GITHUB_ACCOUNTS / `clone` w/ something `hub` offers, if
# applicable

function clone_with_auth_retry() {
    # $1 JUST the <account>/<repo> part of the URL
    # $2 (optional) path to clone the repo
    if [ -z "$1" ]; then
        echo "clone_with_auth_retry missing first argument!"
        return 2
    fi
    # A note on git+Github's behavior: https auth will succeed without requiring
    # a username and password for public repos, while SSH will fail for public
    # repos (it seems. i used vagrant to test and maybe i just didn't specify
    # ssh key password correctly, and then it got saved?)

    # TODO if SSH fails and HTTPS works, maybe set some global (to parent shell)
    # visible env var to indicate which auth to use (so that future calls to
    # this fn, from same shell, don't need to try ssh?)

    # TODO (on reboot, because my ssh agent is usually running as my config
    # is now) test what the exit code / output is if SSH key password is just
    # entered incorrectly. we may not want to treat that as below.

    account="$(awk -F/ '{print $1}' <<< $1)"
    if [ -z "$account" ]; then
        echo "Do not start first argument of clone_with_auth_retry with /"
        return 4
    fi

    if [ -z "$2" ]; then
        # could also check this is non-empty / valid
        dir="$(awk -F/ '{print $2}' <<< $1)"
    else
        dir="$2"
    fi
    # Checking this b/c git still gives exit code 128 in this case it seems.
    if [ -e "$dir" ]; then
        # technically it wouldn't fail if it was an empty dir, but whatever
        echo "clone would fail because ./$dir already exists!"
        return 5
    fi

    url="https://github.com/$account"
    # Assuming that account will exist if in this array.
    # https://stackoverflow.com/questions/3685970
    if [[ ! " ${MY_GITHUB_ACCOUNTS[@]} " =~ " $account " ]]; then
        # https://stackoverflow.com/questions/38806153
        if curl -s --head  --request GET ${url} | grep "404 Not Found" > /dev/null
        then
            echo "Github account at: $url did not exist!"
            return 3
        fi
    fi

    # TODO find some nice way to also capture stderr? this doesn't work.
    #out="$(git clone git@github.com:$1 $2 2>&1)" # (just has stdout i think)
    # see: https://stackoverflow.com/questions/13806626

    # git exit code is 128 in both the case when cloning a (public, in test)
    # repo ssh fails AND the case where the repo does not exist
    local out="$(git clone git@github.com:$1 $2)"
    #out="$(git clone git@github.com:$1 $2 2>/dev/null)"
    #out="$(stderr_too 'git clone git@github.com:'$1' '$2)"
    c1_exit_code="$?"
    #echo "c1: $c1_exit_code"

    # TODO could refactor a bit using a fn that implements checking logic
    # below and takes full url

    if [ "$c1_exit_code" -eq 0 ]; then
        # TODO any way to do this, while preserving both stderr and stdout?
        # (printing to correct streams)
        printf "$out"

    elif [ "$c1_exit_code" -eq 128 ]; then
        #echo "git clone with ssh auth failed. retrying with https..."
        # originally wanted to fallback to https, but i found it annoying
        # that it hangs (prompting for password) if repository doesn't exist
        # (b/c it could be private, presumably)
        # TODO maybe still fallback to https somehow, as long as we can not hang
        #git clone https://github.com/$1 $2

        #echo "git clone with ssh auth failed. retrying with no auth..."
        local out="$(git clone git://github.com/$1 $2)"
        #out="$(git clone git://github.com/$1 $2 2>/dev/null)"
        #out="$(stderr_too 'git clone git://github.com/'$1' '$2)"
        c2_exit_code="$?"
        if [ "$c2_exit_code" -eq 0 ]; then
            printf "$out"
            return

        elif [ "$c2_exit_code" -eq 128 ]; then
            #echo "git clone with no auth failed"
            #echo "c2: $c2_exit_code"
            return $c2_exit_code

        else
            printf "$out"
            echo "unexpected exit code ($c2_exit_code) from git clone."
            return $c2_exit_code
        fi

    else
        printf "$out"
        echo "unexpected exit code ($c1_exit_code) from git clone. aborting."
        return $c1_exit_code
    fi
}

# TODO TODO accept full url and handle that
# (so i can just copy paste url from github w/o having to edit it, negating
# convenience)
function clone() {
    # $1 <account>/<repo> or <repo> if under MY_GITHUB_ACCOUNTS
    # $2 (optional) path to clone the repo

    n_slashes=$(grep -F -c '/' <<< "$1")
    if [ -z "$1" ]; then
        # TODO maybe move to usage message (accessible how?) and just
        # fail w/ a simpler message like below?
        printf "First argument must be <account>/<repo> or, if under one of "
        printf "MY_GITHUB_ACCOUNTS, <repo> also works\n\nMY_GITHUB_ACCOUNTS:\n"
        printf '%s\n' "${MY_GITHUB_ACCOUNTS[@]}"
        return 1

    elif [ "$n_slashes" -eq 2 ]; then
        # TODO maybe add more descriptive usage message here
        echo "Too many arguments (expected 1-2)"
        return 1

    elif [ "$n_slashes" -eq 1 ]; then
        repo_url="$1"
        clone_with_auth_retry $1 $2

    elif [ "$n_slashes" -eq 0 ]; then
        for account in "${MY_GITHUB_ACCOUNTS[@]}"
        do
            # TODO print counter (like (1/2))?
            echo "Trying account: $account"
            clone_with_auth_retry $account/$1 $2
            ec="$?"
            if [ "$ec" -eq 0 ]; then
                return
            # The code I return if directory already existed.
            elif [ "$ec" -eq 5 ]; then
                return $ec
            fi
        done
    fi
}
# TODO TODO maybe overload this to "clear" in no-arg case
alias cl="clone"

# TODO add another alias that runs this, but only on repos modified within some
# reasonable time window (maybe ~1week / ~1mo)
# Need to follow the install instructions for this in my dotfiles README
alias mg='mgitstatus -w --no-push --no-pull --no-upstream --no-uncommitted --no-untracked --no-stashes -e -d 1; mgitstatus -e'
# Also do a fetch on each repo (can be slower)
alias mgf='mg -f'

function reload_bashrc() {
    # Might still want to do this, in cause sourcing screws with other
    # parts of the venv / conda env...
    #if in_virtual_env; then
    #    echo "Some virtual environment active. Deactivate and recall."
    #    return 1
    #fi
    echo "reloading ~/.bashrc"
    # Because virtual envs fuck with PS1, though at least some other
    # parts still seem to work after sourcing ~/.bashrc
    curr_ps1=$PS1
    source ~/.bashrc
    PS1=$curr_ps1
}

# TODO maybe define this / related to deactivate / reactivate any virtual envs
# (or more easily, just warn and NOOP if one is active) first, so that ps1
# / other env stuff is preserved
#alias sb="echo 'reloading ~/.bashrc'; source ~/.bashrc"
# TODO maybe 'set +o xtrace' before (to disable) and restore whatever value we had for
# xtrace after?
alias sb="reload_bashrc"
alias r="sb"

alias o="sudo"

alias rb="sudo reboot"

alias wow='wine /media/tb/Games/wow-4.3.4/wow_434.exe 1>/dev/null 2>/dev/null &'

alias lmms='$HOME/src/lmms/build/lmms'
alias lac='LAC'

#alias plots='scp tom@eftm.duckdns.org:~/lab/hong/src/*html .'
alias fiji='$HOME/Fiji.app/ImageJ-linux64'

alias f='cd ~/shared/FoundryVTT'

# TODO get vim formatting (e.g. gq) to work with bash comments (recomment new
# lines) + string handling + command breaking


function wake_host() {
    # NOTE: input not actually used now cause iface and mac are hardcoded for
    # nas... (since i couldn't figure out how to look them up when the computer
    # was off)
    host_to_wake="$1"
    if ! host "$host_to_wake" > /dev/null 2>&1; then
        echo "host does not exist"
        return 1
    fi

    # TODO shouldn't there be some way to query pfsense for the mac address
    # (the DHCP server that has the static mapping...), rather than relying on
    # the ARP cache?

    # iface and host_mac can be retrieved by "arp -an <hostname / ip>"
    # if this computer talked to that host recently
    # This file should have these two fields on the same line, whitespace
    # separated.
    iface_and_hostmac=`cat ~/.nas_iface_and_hostmac`
    iface=$(echo $iface_and_hostmac | awk '{print $1}')
    hostmac=$(echo $iface_and_hostmac | awk '{print $2}')

    #echo "iface: $iface"
    #echo "hostmac: $hostmac"

    sudo etherwake -i $iface $hostmac
}
alias wakenas='wake_host nas'

alias s='ssh'
alias sx='ssh -X'
alias sshx='ssh -X'

alias sa='ssh atlas'
alias sax='ssh -X atlas'

alias scpr='scp -r'

alias tm='tmux'
# TODO some shortcut for attaching to the most recent (anything like this already built
# in to tmux?)? (like w/ no args)
# TODO TODO convenience wrapper to open a terminal, ssh, and tmux attach to each session
# in tmux list-sessions output?
alias ta='tmux attach -t'
alias tls='tmux list-sessions'

# Just obfuscating a bit, in case somebody is scraping Github for SSH aliases...
# There's probably a better way.
duck_pfx="eftm."
shsp1=12
shsp2=48
alias bb="ssh tom@${duck_pfx}duckdns.org -p ${shsp1}${shsp2}"
alias bbx="ssh tom@${duck_pfx}duckdns.org -p ${shsp1}${shsp2} -X"

# TODO what was this for again? forcing use of password? but why?
alias snk='ssh -o PubkeyAuthentication=no'
#alias atty='stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost
#-onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon
#-crtscts -hupcl'

alias x='xdg-open'

alias rs='rsync -auvP'
complete -F _complete_alias rs

alias cpconf='cp $HOME/catkin/src/multi_tracker/examples/sample_data/config_20160412_134708_N1.py .; mv config_20160412_134708_N1.py config_`ls -p | grep bag | sed s/_delta_video.bag//g`.py'


# TODO actually re-setup my windows ~/.bash[rc/_aliases], so it works for git
# bash (in addition to still working for WSL)

# Trying to detect Git Bash on Windows, to alias python so it works on
# my Windows 10 setup, using the Python 3.8 that I think I installed from
# the Windows store?
if [[ "$(uname)" =~ ^MINGW.*  ]]; then
    # A crude empirical test to distinguish Git Bash from the MINGW that came
    # with MSYS2, Windows 10 5/24/2020
    if printenv | grep -i git | grep -i path -q; then
        # https://stackoverflow.com/questions/32597209
        alias python='winpty python'
        alias python3='winpty python3'
    fi
fi

# TODO might want to test some other way if the RHS of either could be something
# other than empty or an executable file (what -x tests for) (for example, an
# alias)
# TODO TODO check all other uses of command across my dotfiles don't make the
# same mistake
if [ -x "$(command -v python3)" ]; then
    if ! [ -x "$(command -v python)" ]; then
        alias python='python3'
    fi
fi
alias py='python'
alias py3='python3'
alias p='python'
alias p3='python3'
alias p3m='python3 -m'
# TODO also print full python version number
# TODO also print real path if any symlinks
alias wp='printf "\`which python\`="; which python'
alias wp3='which python3'

function which_module() {
    # TODO tab completion w/ installed modules
    # TODO NOOP/err if no non-empty arg passed (or >1)
    # TODO check module is importable / suitable err if not
    # TODO TODO deal w/ cases like:
    # AttributeError: module 'hong2p' has no attribute '__file__'
    # (caused by editable? encountered calling from natural_odors venv)
    local pyfile=`python -c "import $1; print($1.__file__)"`
    echo "__file__: $pyfile"
    # TODO skip everything below if above failed

    # TODO test behavior if there is a symlink somewhere in path +
    # probably indicate if there is a symlink (also printing target)

    # TODO TODO print whether managed by pip (conda too?) (and version)
    # + install url? possible?
    # (w/ ssh, pip freeze in conda seems to include hash... https in regular
    # venv seems not to)

    # TODO TODO instead some command that searches using full path, rather than
    # module name?
    echo "\`pip freeze\` entry: $(pip freeze | grep -i $1)"

    # TODO TODO only do conda list after testing a (non-base?) conda env is
    # active (expected list to be empty in regular venv, but it's not)

    # https://stackoverflow.com/questions/16391208
    # TODO test w/ [editable/not] folders
    echo "last modified: $(date -r $pyfile)"

    # maybe just if not implementing symlink checking...
    # https://stackoverflow.com/questions/545387
    # TODO suppress final '  -'
    #echo "$(find $pyfile -type f -print0)"
    # TODO TODO test in case where pyfile is a file, rather than a directory.
    # (e.g. w/ `which_module chemutils` from natural_odors venv)
    echo "hash: $(find $pyfile -type f -print0 | sort -z | xargs -0 sha1sum | sha1sum)"
}
alias wm='which_module'

alias pi='pip install'

alias pie='pip install -e'
complete -d pie

alias pir='pip install -r'
complete -f -o plusdirs -X '!*.txt' pir

# TODO TODO change this to a function that uninstalls python package(s) defined
# in setup.py in current directory if no argument is specified.
# (and maybe make another alias that combines this with subsequently
# reinstalling, via 'pip install .'?). could probably shortcut to just
# uninstalling the name of the current directory / directory name of git repo
# root. (pr is a builtin tool related to printing text files, but probably still
# fine to use that for this alias...)
# Only this one seems to need confirmation (and thus -y), for some reason.
# TODO custom tab completion (pip uninstall doesn't seem to have it either, at least by
# default)
alias pu='pip uninstall -y'
alias pup='pip install --upgrade pip'

# Invoking a Python script this way will avoid BdqQuit traceback on Ctrl+D at an
# `ipdb.set_trace()` breakpoint https://stackoverflow.com/questions/34914704
# NOTE: pretty sure this wasn't working correctly (some prints leading up to breakpoint
# weren't there. maybe it was breaking at start of script and i just needed to continue
# or something?)
#alias pd='python -m ipdb'

# "sphinx test" (sb "build" was taken by source bash)
alias st='make html && xdg-open build/html/index.html'

# TODO also alias mf3 to a version of that that forces python3
# (maybe have former intead just always default to python3 if available though?)
#alias mf="python -c 'import '"

# TODO TODO maybe make something to count frequency of commands you use and make
# like a huffman kind of tree from it (to assign shorter codes, maybe also
# weight by keys in resting position, w/ weights falling off further away?
# someone have a model for the ergonomic impact of each key?)

# Trying this out for a bit:
# TODO TODO wrap w/ a fn or something can can check if we have recently called
# update, to avoid need for u?
alias u='sudo apt update'
alias i='sudo apt install -y'
alias saa='sudo apt autoremove'

#alias i='ipython'
alias ipy='ipython'
alias ipy3='ipython3'

alias j='jupyter notebook'

# TODO also:
# - grep for '@profile' or the like (ideally uncommented...) (would need to move to fn)
#   and don't call (but warn) if missing
# - install line_profiler if missing?
# - delete output file?
# TODO also add whatever args it took to get units in to seconds?
alias prof='kernprof -l -v'
# TODO wrapper to get the biggest offenders / remove many consecutive non-run lines?
# TODO maybe an alias to [prompt and] remove each '@profile' / similar that exists in
# python files in current tree?

alias cm='cd ~/catkin && catkin_make'

# if this ever causes problems with logs, can also include ROS_LOG_DIR=/home/user/.ros/log
# TODO move this alias to hong-lab-system dependent deployment file,
# likely as w/ some env vars
#alias roslaunch='ROS_HOME=`pwd` roslaunch'
# TODO why does tab completion not seem to work with this one? can i make it?
alias rl='ROS_HOME=`pwd` roslaunch'

# TODO had alias e=exit on blackbox, and c=clear, and i might prefer those
#alias e='cd $EXP_DIR'
#alias cdc='cd ~/src/al_imaging'
# d=data
#alias cdd='cd /media/threeA/Tom/flies/'
# & cdd that uses env var to go to data

# TODO rename to rd?
alias fd='roscd'

# TODO TODO TODO replace this check of completion_loader w/ something that
# works for functions (what _completion_loader is here, as you can see w/
# "type _completion_loader" on blackbox (which has it defined))
# ('type' might work. try to use a test as shell indep as possible.)

# TODO TODO related to above, maybe make my own function that tests if something
# is either an executable OR an alias that points to one? not sure yet whether
# i want to include anything else, like bash functions...

#if [ -x "$(command -v _completion_loader)" ]; then
#    # TODO idk why this worked in one terminal, but now seems to have issues in
#    # new ones...  see where .bash_aliases is sourced in whatever sources
#    # completion stuff i suppose
#
#    # TODO _completion_loader isn't a ROS thing, though, right?
#    # should i also be testing for ROS presence first?
#    _completion_loader roscd
#
#    # got from `complete -p roscd`
#    complete -o nospace -F _roscomplete_sub_dir fd
#fi

# TODO change this into a function so it can work for whichever arduino version
# we actually have installed in my home directory
# (and that doesn't risk opening multiple like it does with asterisk)
alias arduino='~/arduino-*/arduino'

# Library Update
# TODO maybe find whichever arduino is actually installed?
alias lu='cd $HOME/arduino-1.8.3/libraries rm -rf ros_lib && rosrun rosserial_arduino make_libraries.py .'

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

if ! [ -x "$(command -v vi)" ]; then
    alias vi='vim'
fi
alias v='vi'
alias sv='sudo vi'

alias black='black --skip-string-normalization'

# It seems if it was saved w/ a diff version of python or something, nothing is
# printed? kind of odd, considering it worked with 3 and I thought i would have
# saved the temporary mappings I tested it on in 2 (ROS). Syntax for Python 2?
alias pickle='python -mpickle'
alias pickle3='python3 -mpickle'

alias ssid="nmcli -t -f active,ssid dev wifi | egrep '^yes' | cut -d\' -f2"

# TODO TODO try to fix MINGW displaying of python venv in PS1? easy enough
# to be worth it? (for MSYS2 stuff)

alias e='echo'

# Starting vim in insert mode with paste option set.
alias vip="vim +startinsert -c 'set paste'"
alias vp="vip"

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
# TODO try to make something like 'sr' that accepts one dir input and make it tab
# completable based on stuff actually in ~/src
alias sr="cd ~/src"
alias cs="cd ~/src"

alias dot="cd ~/src/dotfiles && git status"
# Not using `do` because that is some other keyword.
alias dt="dot"

# Too easy to tab complete too `script`, which is annoying.
alias scripts="cd ~/src/scripts && ls"
alias scr="scripts"

alias m='man'

# I currently have this python script under GithubCloner in my ~/src/scripts
# repo, and add it to PATH in the portion of ~/.bashrc that adds ~/src/scripts
# submodules to PATH.
alias githubcloner="githubcloner.py --prefix-mode directory --output-path ."

# TODO TODO does the --include-org-members actually still work? how?
# is output from it and --include-gists included in --echo-urls output?
# (when testing w/ ejhonglab, it at least doesn't seem to be cloning MY
# public stuff...)

# TODO maybe test whether i am a member of this org? or whether i can
# authenticate? and then pass --include-authenticated-repos?
# or maybe just modify githubcloner.py (if this isn't already the default
# behavior) so that it just doesn't include them if the flag is passed and
# we can't authenticate for them?
_pubmsg="Downloading all PUBLIC repos of specified "

# TODO maybe add a flag to not clone forks? or skip if they have under some
# threshold amount of commits on top of upstream?

# TODO TODO adapt githubcloner / make my own tools to keep all of the things
# up-to-date WITHOUT deleting anything (prevent git history from being rewritten
# too / warn / copy if it would be)

# TODO TODO if user following from orgs works, but relies on the "people"
# section of the org page, maybe re-implement in a way that scrapes names and
# emails from commit logs and then tries to find their githubs that way

# TODO TODO TODO go back over what i have downloaded in
# ~/src/github_organizations, and check for missing / incomplete repos
# (even w/o errors printed, some runs definitely had missing stuff.
# see bocklab, for one example.) is threading the issue? pass that CL arg
# w/ value 1, to disable that?

# Based on this answer that says gists can only really be under an organization
# account if they were already under some personal account that got promoted to
# an organization, the --include-gists option should never really matter for
# organizations.
# https://stackoverflow.com/questions/20647454
alias cloneorg="echo \"${_pubmsg}org\"; githubcloner -org"

# TODO maybe just test whether user is among MY_GITHUB_ACCOUNTS, if not
# implementing more general support that would also apply to orgs, as in
# todos above?
alias cloneuser="echo \"${_pubmsg}user\"; githubcloner -u"

# TODO maybe add an alias (or put in my scripts repo?) for what Victor Yarema
# describes here: https://stackoverflow.com/questions/33024085 for displaying
# which step along the rebasing progress you are

# TODO need to update these if i want something that works w/ multiple remotes
# (like origin + upstream) (see open_repo_in_browser above too)
# Uses a script in my scripts repo.
alias gitgit="git remote -v | change_git_auth.py g | xargs git remote set-url origin"
alias githttps="git remote -v | change_git_auth.py h | xargs git remote set-url origin"
alias gitssh="git remote -v | change_git_auth.py s | xargs git remote set-url origin"

# TODO TODO either by default or w/ an alias that adds one letter, add that flag
# to treat all binary files as not matching (to speed up matching in cases where
# most of the data in a directory tree is in binary files...)
# TODO should i just exclude all hidden folders? how?
# TODO factor grep exclude dirs into bash array and expand here (or can i just
# use an env var that grep would use directly? one exist?)

# Need the --color=auto because I don't think aliases can be redefined to extend
# what an alias of the same name defines (otherwise the definition in ~/.bashrc
# that is there by default would add this argument)
alias grep="grep --color=auto --exclude-dir=.direnv --exclude-dir=site-packages --exclude-dir=.git"
alias gr="grep"
# Like -r but also follows symbolic links
alias grr="grep -R"

# TODO TODO TODO write last pattern grepped to a file and then made vim shortcut to read
# that file and go search for that pattern (maybe <leader>n?) (might need to convert the
# search pattern format?)

# TODO add a version of this that only searches files tracked by git
# (to automatically avoid any build artifacts, like python egg stuff, etc)
# Main difference between -r and -R seems to be that -R follows symlinks.
# Using this syntax for multiple exclude-dir b/c it's friendly with grepym.
# TODO kinda want --color=always, but for some reason it seemed to break the actual
# matching in a pipe to a subsequent grep, which is more important than the
# colors...
# e.g. `grepy --color=always lam | grep -v lambda | wc -l` had the same number of lines
# as without the `| grep -v lambda` part...
alias grepy="grep -R --include=\*.py --exclude-dir=site-packages --exclude-dir=.eggs --exclude-dir=venv"
alias gpy="grepy"
# This will lookup and use the alias definition above at runtime.
alias grepym="grep_py_in_my_repos.py"
alias gpym='grepym'

# TODO TODO alias to cd to a folder and then vi any .py files 1) w/ prefix of
# foldername or 2) lone .py files

alias dlwebsite="dlwebsite.py"

alias lr='ls -ltr'

# Since I already have muscle memory for typing 'df -h'
# The goal is just to include the lines for real storage devices (the things I generally
# care about).
alias dfh="df -h | grep -v '/dev/loop' | grep -v 'tmpfs' | grep -v 'udev'"

alias brc="vi ~/.bashrc"
alias br="vi ~/.bashrc"

alias vrc="vi ~/.vimrc"
# vr saved for vagrant reload

alias ba="vi ~/.bash_aliases"

alias mi="vi ~/src/scripts/movein.sh"

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

# TODO maybe available_space command or something like that to parse
# df -h output (since on stuff using snap, the df -h output is kinda polluted)
# (or just pass diff args to only get parition containing root / home)

# https://stackoverflow.com/questions/4996090
alias R='R --no-save'

# This is the drive I have mounted at /home/tom/shared
export FOUNDRY_VTT_DATA="/home/tom/shared/FoundryVTT"
export FOUNDRY_SOUNDS="$FOUNDRY_VTT_DATA/Data/sounds"

alias fvd="cd $FOUNDRY_VTT_DATA/Data"

# since generate normalize-audio didn't seem to work on .ogg despite saying it
# should...
alias normalize_foundry_oggs="find $FOUNDRY_SOUNDS -name '*.ogg' -type f -exec normalize-ogg {} \;"

# Just to quickly check if internet is working by pinging one of the Google DNS
# servers.
alias p8="ping -c 2 8.8.8.8"

# not sure which of these i want to keep... probably not ALL of them
# `grep http` is just to cut # of lines in two
# Just redirecting stderr to /dev/null to silence the warning about apt not having a
# stable CLI. Could in theory cause problems.
# TODO it's not because i misconfigured my system that i have duplicate ppa entries (on
# blackbox) without the `... | sort -u` part, right (2 consecutive of each)?
# (could also *maybe* replace `apt` w/ `apt-cache` if i want to not silence stderr?)
alias ppalist="apt policy 2>/dev/null | grep ppa | grep http | cut -d' ' -f3 | cut -d'/' -f4,5 | sort -u"
alias ppals='ppalist'
alias ppa='ppa_list'
alias ppas='ppa_list'

# TODO try to include the answer from here: https://askubuntu.com/questions/447129
# ...in a function or something. it lists all packages installed from all PPAs, though
# maybe with *some* cases not handled correctly. or just escape the quotes correctly and
# leave it as an alias. could maybe grep its output to determine what is installed for
# one particular ppa (unless there is a better way?)

# TODO add alias 'manifest' to generate URL that should exist for *.manifest file of
# current ubuntu version (maybe always using the latest Z version, for version X.Y.Z?),
# and open it in a web browser (/ print it to terminal for grepping)?

# NOTE: sometimes it seems this will seem to say something is installed from a
# dependency even if it was (I'm pretty sure) manually installed. Ex: git from the ppa
# on blackbox. It says 'tldr' depends on it (maybe it was installed after?)
alias aw='aptitude why'

# TODO maybe parse output and only print url/line (from the "Version table:") matching
# the "Installed: " version?
# 'apt policy <x>' is useful for finding out which repository package <x> came from.
# https://askubuntu.com/questions/8560
alias ap='apt policy'

# TODO make an alias for this (count_files_in_subdirs or something)
#du -a | cut -d/ -f2 | sort | uniq -c | sort -nr

# TODO TODO maybe also have this detect if it is a tar.bz2 and unzip it if so
# TODO replace verbose output with tqdm-like progress bar? optionally? both,
# ideally...
# TODO TODO TODO test
# TODO TODO TODO fix! seems broken! might have deleted some of my rotation al
# data...
#function bzip2_directory() {
#    # TODO any other tar options i want? (to preserve permissions and times and
#    # stuff) (should do all that by default, but maybe check...)
#    # The f actually needs to come last or it won't work right.
#    bz2_output="$1.tar.bz2"
#    if [ -f "$bz2_output" ]; then
#        echo "$bz2_output already existed"
#        return
#    fi
#    tar -cjvf "$bz2_output" "$1"
#}
#alias bzdir='bzip2_directory'

alias sts='showsync'

# Not that this is exactly same output format as `ifconfig` had, but apparently
# `ifconfig` has long been deprecated in favor of `ip`, so doing this rather than
# installing `net-tools` to delay the inevitable.
# https://askubuntu.com/questions/1031640
# TODO include message warning + saying what we are actually calling?
alias ifconfig='ip -c a'


# For quickly generating prefixes for filenames, e.g.:
# $ echo `ds`out.txt
# 2021-04-25_out.txt
# $ echo `ts`out.txt
# 2021-04-25_200431_out.txt

# The sed command here is just to add a trailing underscore.
# [d]ate [s]tring
alias ds="date --iso-8601 | sed -e 's/$/_&/'"
# Assumes we can rely on the first 19 characters always having what we want, and 'T'
# always being the placeholder between day and hour parts.
# TODO lookup whether there are exceptions to these assumptions.
# Based partially on https://unix.stackexchange.com/questions/120484
# [t]ime [s]tring (/ [s]tamp)
alias ts="date --iso-8601=seconds | cut -c1-19 | sed -e 's/T/_/' -e 's/://g' -e 's/$/_&/'"

# Same as above, but puts output in clipboard.
alias cds='ds | xclip -sel clip && echo "Date prefix in clipboard"'
alias cts='ts | xclip -sel clip && echo "Timestamp prefix in clipboard"'


alias lns="ln -s"

# [c]lip [i]n
alias ci='xclip -sel clip'

# TODO should i also use '-sel clip' here or is that only relevant on input? not clear
# if it ever really matters for me...
# [c]lip [o]ut
alias co='echo "writing clipboard contents"; xclip -o >'


# TODO TODO integrate setup of gnome-terminal Ctrl+a select all shortcut to my other
# shortcut/ui setup scripts. currently i set it manually via: Edit -> Preferences ->
# (on left pane) Shortcuts -> (double?) left click space under "Shortcut Key" column for
# "Edit -> Select All" row
function copy_terminal_contents() {
    # NOTE: requires this shortcut to be set up (currently done manually; see comment
    # above) in gnome-terminal.
    xdotool key Ctrl+a
    xdotool key Ctrl+Shift+c
}
alias csb='echo "copied terminal scrollback to clipboard"; copy_terminal_contents'

# TODO TODO python scripts to parse output to get:
# - the last python traceback (-> also parse file+linenos -> integrate w/ vim somehow?)
# - the last command (command + all output. ignore any empty following prompts.)
# (-> aliases)
function write_terminal_contents() {
    if ! _check_one_nonexistant_arg "$@"; then
        echo "exiting"
        return 1;
    fi
    copy_terminal_contents

    echo "writing terminal scrollback to $1"
    xclip -o > "$1"
}
alias wsb='write_terminal_contents'

alias makej='make -j$(nproc --ignore=2)'
alias mj='makej'

alias md5='md5sum'
alias sha256='sha256sum'


AL_PAIR_GRIDS_CONDA_ENV="suite2p"

# TODO make accept argument + add completion like c1/2/etc if i end up using enough
# Requires hong2p to be setup in current shell environment / python
function 2p() {
    conda activate $AL_PAIR_GRIDS_CONDA_ENV
    cd "$(hong2p-data)/$1"
}
function 2pr() {
    conda activate $AL_PAIR_GRIDS_CONDA_ENV
    cd "$(hong2p-raw)/$1"
}
function 2pa() {
    conda activate $AL_PAIR_GRIDS_CONDA_ENV
    cd "$(hong2p-analysis)/$1"
}

function print_2pa() {
    if [ -n "$CACHED_HONG2P_ANALYSIS_DIR" ]; then
        echo $CACHED_HONG2P_ANALYSIS_DIR
        return
    fi

    if ! [ "$CONDA_DEFAULT_ENV" = "$AL_PAIR_GRIDS_CONDA_ENV" ]; then
        conda activate $AL_PAIR_GRIDS_CONDA_ENV
    fi
    # Saving to environment variable before printing so we can reply faster
    CACHED_HONG2P_ANALYSIS_DIR=$(hong2p-analysis)
    echo $CACHED_HONG2P_ANALYSIS_DIR
}

complete -C "_dir_completion_via_dir_fn hong2p-data" -o plusdirs -o nospace -S '/' 2p
complete -C "_dir_completion_via_dir_fn hong2p-raw" -o plusdirs -o nospace -S '/' 2pr

complete -C "_dir_completion_via_dir_fn hong2p-analysis" -o plusdirs -o nospace -S '/' 2pa
# TODO why does the above work but not this? was trying to not need to conda activate
# first...
#complete -C "_dir_completion_via_dir_fn print_2pa" -o plusdirs -o nospace -S '/' 2pa

function suite2p_combined_view() {
    2pa

    local prefix
    if [ -z "$1" ]; then
        prefix="."
    else
        prefix="$1"
    fi
    # --statfile only available in my fork
    suite2p --statfile "$prefix/suite2p/combined/stat.npy" &

    # just estimating, but seems fine so far on my home computer
    local initial_delay_s=3.0

    sleep $initial_delay_s

    # re: xdotool's support for delays between keys:
    # https://askubuntu.com/questions/1098762

    # w=mean, r=correlation map, t=max
    # k=color ROIs by correlation w/ selected
    # b=hide "neuropil" trace
    # n=hide "deconv"
    xdotool key r k b n

    # TODO wait for suite2p to be closed before exiting
    # (to preserve blocking behavior suite2p_and_dff relied on to kill image viewer
    # after suite2p was closed)
}
alias ss='suite2p_combined_view'

function suite2p_and_dff() {
    pkill eog
    pkill suite2p

    # this puts us in the directory we need to be in for the svg picking to work
    # correctly
    2pa
    # assuming this is a relative path from analysis intermediate root to a subdir
    # corresponding to one experiment
    cd $1

    # NOTE: this won't be the image w/ the biggest response for reverse order stuff
    highest_concs_mix_svg="$(ls -Art `python -c 'import os; parts = os.getcwd().split("/"); print("/home/tom/src/al_pair_grids/svg/" + "_".join(parts[5:]))'`/*_trials.svg | tail -n 1)"
    # eog is default image viewer. xdg-open would also open it via eog.
    # calling it w/ eog w/o additional arguments or & blocked tho and i dont want that

    if [ -f "$highest_concs_mix_svg" ]; then
        xdg-open $highest_concs_mix_svg
    else
        >&2 echo "$highest_concs_mix_svg not a file!"
    fi

    suite2p_combined_view "$1"

    # Taken from left part of connected screens in `xrandr` output
    local right_screen_name="DP-4"

    # From output of `wmctrl -l -x` (3rd column)
    #local image_viewer_class="eog.Eog"

    local image_viewer_class="eog"

    # Should be available on the PATH b/c it's in my ~/src/scripts repo
    move_wclass.py $image_viewer_class $right_screen_name

    wmctrl -F -a "$(basename $highest_concs_mix_svg)"

    xdotool key Ctrl+Super+Left
    xdotool key Ctrl+plus
    xdotool key Ctrl+plus

    printf "\n- $1:\n  - \n\n" | xclip -sel clip
    # can only restore if i make suite2p_combined_view block until suite2p is closed
    #pkill eog
}
alias sd='suite2p_and_dff'

alias sl="2pa; ls -ltr */*/*/suite2p/combined/iscell.npy | awk '{print \$6, \$7, \$8, \$9}'"


# rs = my rsync alias defined above (rsync -auvP)
# TODO is another --delete<when> option more efficient?
# Going w/ --delete-after for now b/c presumably I have more time to Ctrl-C it if I
# notice something wrong...
# [t]ransfer [d]ata [h]al
alias tdh='rs --delete-after /mnt/d1/2p_data/raw_data/ hal:~/2p_data/raw_data'
# [t]ransfer [d]ata [h]al, [a]ll
alias tdha='tdh; rs --delete-after /mnt/d1/2p_data/analysis_intermediates/ hal:~/2p_data/analysis_intermediates'

# TODO equivalent aliases to above but for transfering to/from USB stick (including
# stimulus files to corresponding directory on NAS) + to dropbox backup folder
# TODO tho maybe prompt to pause syncing before dropbox one?
# TODO + one to transfer to my home computer


# snap install only one i found that could load .dxf files on 18.04
# install via `sudo snap install inkscape`
alias inkscape='snap run inkscape'

# TODO factor into a fn that does this (w/ deletion behind a prompt!!!) before opening
# the file in vim as normal
function vim_kill() {
    # TODO check $1 exists
    # TODO check .$1.swp exists
    # TODO check that we have actually matched a process before trying to call kill
    kill $(lsof .$1.swp 2>/dev/null | tail -n 1 | awk '{print $2}')
}
alias vk='vim_kill'

# https://stackoverflow.com/questions/48574100
function toggle_xtrace() {
    case "$-" in
        (*x*) set +o xtrace; echo "command echo (xtrace) OFF";;
        (*) echo "command echo (xtrace) ON"; set -o xtrace;;
    esac
}
alias xt='toggle_xtrace'

alias da='direnv allow'

