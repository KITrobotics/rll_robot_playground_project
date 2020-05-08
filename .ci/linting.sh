#!/bin/bash

set -e

CLANG_TIDY_CMD="run-clang-tidy-7.py -checks=-readability-function-size -p /root/catkin_ws/build"
CLANG_TIDY_CHECK_PACKAGES="rll_robot_playground_project"

for package in $CLANG_TIDY_CHECK_PACKAGES
do
    $CLANG_TIDY_CMD/$package -header-filter=$package/*
done

pip -q install 'pylint<2.0.0'
. /root/catkin_ws/install/setup.bash
find * -iname '*.py' | xargs python -m pylint --disable=duplicate-code,too-many-statements
