#!/usr/bin/env bash
#
# Strawberry - libsb
# Copyright 2016-2017 Honda Research Institute Japan. All rights reserved.
#

# Abort on errors
set -e

if [ -z "$NEED_ROOT" ]; then
	echo 'Please set the variable NEED_ROOT to 0 or 1'
	exit 1
fi

# To make sure that the root password promt is shown even if stdout is redirected
# We call this a lot to avoid getting a stuck script waiting for the password.
function check_root
{
	sudo true
}

# Pass what you are going to do
function new_command
{
	echo -e "\033[0;34m$1\033[0m"
	if [ "$verbose" = 1 ]; then
		echo ""
	else
		echo -en "[\033[0;34m....\033[0m]: "
	fi
	
	sleep 2
	
	if [ "$NEED_ROOT" = 1 ]; then
		check_root
	fi
}

# The more info about what we are doing (updates the message only)
function ret_info
{
	if [ "$verbose" = 1 ]; then
		echo -e "[\033[0;34mINFO\033[0m]: $1"
	else
		echo -ne "\r\033[K[\033[0;34m....\033[0m]: $1"
	fi
	
	sleep 1
	
	if [ "$NEED_ROOT" = 1 ]; then
		check_root
	fi
}

# Pass a message to show along with the warning message.
function ret_warn
{
	if [ "$verbose" = 1 ]; then
		echo -e "[\033[0;33mWARN\033[0m]: ${1:-"Non-critical error"}" 1>&2
	else
		echo -e "\r\033[K[\033[0;33mWARN\033[0m]: ${1:-"Non-critical error"}" 1>&2
	fi
	
	sleep 5
	
	if [ "$NEED_ROOT" = 1 ]; then
		check_root
	fi
}

# Pass a message to show along with the error message. It will abort the execution
function ret_critical
{
	if [ "$verbose" = 1 ]; then
		echo -e "[\033[0;31mFAIL\033[0m]: ${1:-"Command failed"}" 1>&2
	else
		echo -e "\r\033[K[\033[0;31mFAIL\033[0m]: ${1:-"Command failed"}" 1>&2
	fi
	
	exit 1
}

# The command was successful (pass again what you did)
function ret_ok
{
	if [ "$verbose" = 1 ]; then
		echo -e "[\033[0;32mDONE\033[0m]: ${1:-"Command successful"}"
	else
		echo -e "\r\033[K[\033[0;32mDONE\033[0m]: ${1:-"Command successful"}"
	fi
	
	if [ "$NEED_ROOT" = 1 ]; then
		check_root
	fi
}

# Establish the verboseness level
verbose=

case "$1" in
-v|--v|--ve|--ver|--verb|--verbo|--verbos|--verbose)
    verbose=1
    shift ;;
esac

if [ "$verbose" = 1 ]; then
    exec 4>&2 3>&1
    echo -e "Script set to verbose mode.\n"
else
    exec 4>/dev/null 3>/dev/null
fi

if [ "$NEED_ROOT" = 1 ]; then
	sudo echo "Sudo priviliges received"
fi

# Detect the package manager
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac

function try()
{
    [[ $- = *e* ]]; SAVED_OPT_E=$?
    set +e
}

function throw()
{
    exit $1
}

function catch()
{
    export ex_code=$?
    (( $SAVED_OPT_E )) && set +e
    return $ex_code
}

function throwErrors()
{
    set -e
}

function ignoreErrors()
{
    set +e
}