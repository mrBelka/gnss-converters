#!/bin/bash

# Copyright (C) 2018 Swift Navigation Inc.
# Contact: Swift Navigation <dev@swiftnav.com>
# Run Travis setup

set -ex
set -o errexit
set -o pipefail

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
    make clang-tidy-all && check_tidy_errors
    cd ../
    cd ../
}

if [ "$TESTENV" == "lint" ]; then
  ./c/travis-clang-format-check.sh
else
  build_c
  build_haskell
fi

