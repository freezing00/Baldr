#!/bin/sh
#强制退出代码
echo 'nvidia' | sudo -S echo "Baldr"
PID=`pgrep Baldr` 
#echo $PID
sudo kill -9 $PID
