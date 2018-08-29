#!/bin/bash

# Copyright (C) 2018 Swift Navigation Inc.
# Contact: Swift Navigation <dev@swiftnav.com>
# Run Travis setup

set -ex
set -o errexit
set -o pipefail


#************************************************************************
# UTILITY FUNCTIONS
#************************************************************************
check_format_errors() {
  if [[ $(git --no-pager diff --name-only HEAD) ]]; then
    echo "######################################################"
    echo "####### clang-format warning found! Exiting... #######"
    echo "######################################################"
    echo ""
    echo "This should be formatted locally and pushed again..."
    git --no-pager diff
    travis_terminate 1
  fi
}

function build_haskell () {
    cd haskell
    stack build --test
    cd ../
}

function build_c() {
    cd c
    mkdir build
    cd build
    cmake ../
    make -j8 VERBOSE=1
    make clang-format-all && check_format_errors
    cd ../
    cd ../
}

if [ "$TEST_SUITE" == "lint" ]; then
  ./c/travis-clang-format-check.sh
else
  build_c
  build_haskell
fi
