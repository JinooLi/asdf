#!/bin/bash

path=$(dirname $BASH_SOURCE)

# launch_base_system <map name>
alias launch_base_system=". ${path}/base_system.sh"

# launch_mppi
alias launch_mppi=". ${path}/mppi.sh"

# launch_slam <map name>
alias launch_slam=". ${path}/slam.sh"