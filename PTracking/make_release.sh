#!/bin/bash

rm -rf include
cp -r src include
cd include
rm CMakeLists.txt Doxyfile
find . -name "*.cpp" -exec rm {} \;
