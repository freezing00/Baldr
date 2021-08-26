#!/bin/sh

#相机驱动加载
echo 'intel' | sudo -S echo "USBDriver"
cd /opt/DahuaTech/MVviewer/module/USBDriver
sudo sh loadDrv.sh

#关闭代码图像显示
cd /home/intel/SD/Baldr/visionData
sed -i 's/useDebugGuiFlag: 1/useDebugGuiFlag: 0/g' codeSet.yml

sleep 1
cd /home/intel/SD/Baldr/build
renum=0
#自动输入密码，启动Baldr
sudo ./Baldr
pwd
while true; do
#当Baldr不在运行时启动Baldr，重启三次就重新开机
	echo 'intel' | sudo -S echo "Baldr"
        PID0=`pgrep Baldr`
        if [ ! "$PID0" ]; then
            sudo ./Baldr
	    echo "restart"
            renum=$(($renum+1))
            echo $renum
	fi        
        if [ $renum -gt 3 ]
        then
	    echo "intel"
            shutdown -r now
        fi  
	sleep 2
done
