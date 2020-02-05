#!/bin/sh
mkdir build/qt_tool
qmake src/qt_tool -o build/qt_tool/Makefile
cd build/qt_tool
make
./qt_tool
