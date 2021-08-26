#!/bin/sh
cd ../../SD/Falcon/
# 强制更新代码
git stash
git checkout feature
git pull
cd Build/
# 编译Faclon
rm *.o
rm Falcon
rm Makefile
qmake ../Falcon.pro -spec linux-g++
make -j4
