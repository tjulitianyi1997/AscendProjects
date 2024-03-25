#!/bin/bash

set -e
rm -rf build
mkdir build
cmake -B build
cmake --build build -j
cmake --install build
./build/main

