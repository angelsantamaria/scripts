#!/bin/bash

SCRIPT_PATH=`realpath "${BASH_SOURCE[0]}"`

export SCRIPTS_PATH=${SCRIPT_PATH%/*}
export PATH=${PATH}:${SCRIPTS_PATH}
