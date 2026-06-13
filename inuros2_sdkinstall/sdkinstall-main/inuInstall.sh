#!/bin/bash

#/* ==================================================================
#* Copyright (c) 2023 - 2099.
#* All rights reserved.
#* Author: wangtao 
#* email: 49705613@qq.com
#* ===================================================================*/


CURRENT_DIR=$(cd  $(dirname $0);pwd)
DIST_DIR="/opt/Inuitive/InuDev/bin"

set busarray;
set portarray;
set devicearray;
set findCameras;
set filterIds;

version=`lsb_release -a | awk -F: '/Release/{print $2}'`

banner()
{
    echo -e "\033[31m   _             _      _            \033[0m"
    echo -e "\033[32m  (_)           (_)_   (_)           \033[0m"
    echo -e "\033[33m   _ ____  _   _ _| |_  _ _   _ ____ \033[0m"
    echo -e "\033[34m  | |  _ \| | | | |  _)| | | | / _  )\033[0m"
    echo -e "\033[36m  | | | | | |_| | | |__| |\ V ( (/ / \033[0m"
    echo -e "\033[38m  |_|_| |_|\____|_|\___)_| \_/ \____)\033[0m"
}

Usage()
{
    banner

    echo -e "\n"
    echo " -------------------------------"

    echo " 1: install single-module SDK!"
    echo " 2: install multi-module SDK!"
    # echo " 3: install ROS1!"
    echo " u: uninstall the SDK!"
    echo " -------------------------------"

    # echo " 5: run ROS1!"
    echo " 6: restart service!"
    echo " -------------------------------"

    echo " f: find cameras! "
    echo " s: set confidence!"
    echo " q: exit!"
    echo " -------------------------------"

    echo -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}
######################################################################################################################
## set params
######################################################################################################################
setConfidence()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m Make sure you understand the confidence mean!!!! \033[0m" 
    echo  -e "\033[33m Make sure you understand the confidence mean!!!! \033[0m" 
    echo  -e "\033[34m Make sure you understand the confidence mean!!!! \033[0m" 
    echo  -e "\033[32m suggest number: 1,3,5,7 \033[0m"

    echo  -e "\033[31m Enter to continue, Ctrl+C to exit! \033[0m"
    read  -p " " anykey
    echo  -e "\033[31m start to set confidence! \033[0m" 

    read  -p "Confidence: " confidence

    orgconfidence1="VALUE="\".*\"" USER_ID=\"30433\""
    replaceconfidence1="VALUE="\"$confidence\"" USER_ID=\"30433\""
    orgconfidence2="VALUE="\".*\"" USER_ID=\"30674\""
    replaceconfidence2="VALUE="\"$confidence\"" USER_ID=\"30674\""

    sudo sed -i "s/$orgconfidence1/$replaceconfidence1/g"  $DIST_DIR/../config/Presets/NU4000C0/DPE_Default.xml
    sudo sed -i "s/$orgconfidence2/$replaceconfidence2/g"  $DIST_DIR/../config/Presets/NU4000C0/DPE_Default.xml

    echo  -e "\033[31m Confidence is set! \033[0m" 
    echo  -e "\033[33m Make it works, need to restart computer!!!! \033[0m" 
    echo  -e "\033[33m Make it works, need to restart computer!!!! \033[0m" 
    echo  -e "\033[33m Make it works, need to restart computer!!!! \033[0m" 

    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    read  -p " " anykey
}

######################################################################################################################
## check
######################################################################################################################

checkCamera()
{
    bus=`lsusb|grep 2959|awk -F ' ' '{print  $2  }'| awk -F ':' '{print $1}'`
    device=`lsusb|grep 2959|awk -F ' ' '{print  $4  }'| awk -F ':' '{print $1}' | sed 's/^0*//g'`

    busarray=($bus)
    devicearray=($device)

    if [ -z ${busarray[0]} ]; then
        echo -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
        echo -e "\033[31m Don't find any Inu Camera! \033[0m"
        echo -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
        exit 1
    else 
        findCameras=${#busarray[@]}
        echo -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
        echo -e "\033[32m Fined $findCameras cameras! \033[0m" 

        for ((i = 0; i<${#busarray[@]};i++)) do

            findbusline=`lsusb -t | awk "/Bus ${busarray[i]:3-2}/{print NR}"`
            searchline=$(($findbusline+1))

            startsearch=1

            portids=()
            deviceline=0
            devicespeed=""

            while [ $startsearch -gt 0 ]; do
                searchresult=`lsusb -t | sed  -n "${searchline}p" | grep "Dev ${devicearray[i]}"`

                if [ "$searchresult" != "" ]; then
                    portarray[i]=`echo $searchresult|awk -F ':' '{print $1}' | awk -F ' Port ' '{print $2}'`
                    deviceline=${searchline}
                    devicespeed=`lsusb -t | sed  -n "${searchline}p" | awk -F',' '{print $NF}'`
                    let startsearch--
                else
                    searchline=$(($searchline+1))
                fi
            done 

            lastportidx=0;
            for((j=${deviceline};j>${findbusline};j--));
            do
                curportidx=`lsusb -t | sed  -n "${j}p" | awk '{print index($0,"Port")}'`
                portid=`lsusb -t | sed  -n "${j}p" | awk -F ':' '{print $1}' | awk -F ' Port ' '{print $2}'`

                if [ $j -eq $deviceline ];then
                    portids+=("$portid")
                    lastportidx=${curportidx}
                else
                    if [ $curportidx -lt $lastportidx ];then
                        portids+=("$portid")
                        lastportidx=${curportidx}
                    fi
                fi
            done

            portIdsStr=""
            for item in "${portids[@]}"; do
                portIdsStr+=".$item"
            done
            portIdsStr=${portIdsStr:1}
            portIdsStr=$(echo "$portIdsStr" | rev)
            busId="$((10#${busarray[i]}))"
            filterIds+=("$busId-$portIdsStr")
                        
            # prompttmp=`usb-devices|grep Bus=${busarray[i]:3-2}| grep Dev#=$'  '${devicearray[i]}`
            # if [  ${#prompttmp} -eq 0 ]; then
            #     prompttmp=`usb-devices|grep Bus=${busarray[i]:3-2}| grep Dev#=$' '${devicearray[i]}`
            #     if [  ${#prompttmp} -eq 0 ]; then
            #         prompttmp=`usb-devices|grep Bus=${busarray[i]:3-2}| grep Dev#=${devicearray[i]}`
            #     fi
            # fi
            # portarray[i]=`echo ${prompttmp#*Cnt=} | awk -F ' ' '{print $1}' | sed 's/^0*//g'`

            echo " Cameras in bus:port:device -> ${filterIds[i]}"
        done

        echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    fi
}

######################################################################################################################
## install
######################################################################################################################

removeSDK()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m start to remove SDK! \033[0m" 

    if [ -f /lib/systemd/system/inuservice.service ];then
        sudo systemctl stop inuservice.service
        sleep 2s
        sudo systemctl disable inuservice.service
        sudo rm /lib/systemd/system/inuservice.service
        sudo rm /lib/systemd/system/inuservice@.service
    else
        echo "continue..."
    fi
    sudo systemctl daemon-reload

    sleep 2s
    inudevsdk=`sudo dpkg -l| grep "Inuitive SDK"|awk -F ' ' '{print $2}'`
    if [[ $inudevsdk != "" ]]; then
        sudo dpkg -P $inudevsdk
    fi
    sudo rm -Rf /opt/bitrock
    sudo rm -Rf /opt/Inuitive*
    sudo rm -Rf /opt/inudev*
    sudo rm -Rf /opt/*.InuSensors
    echo  -e "\033[31m SDK is removed! \033[0m" 
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

installsingleSDK()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m start to install SDK! \033[0m" 

    # install SDK deb
    if [ -f $CURRENT_DIR/inudev*.deb ];then
        sudo dpkg -i $CURRENT_DIR/inudev*.deb
        sudo systemctl stop inuservice.service
    else
        sudo tar -zxvf Inu*.tar.gz -C /opt
        sudo cp $DIST_DIR/inuservice.service /lib/systemd/system
        sudo systemctl daemon-reload 
        sudo systemctl stop inuservice.service
    fi

    sleep 2s

    # add ldconfig
    sudo echo -e '#inu default configuration\n/opt/Inuitive/InuDev/bin' >/etc/ld.so.conf.d/inu.conf
    sudo ldconfig

    # delete some unused files
    sudo rm -Rf /opt/Inuitive/InuDev/include/opencv2
    sudo rm -Rf /opt/Inuitive/InuDev/include/opencv
    sudo rm -Rf /opt/Inuitive/InuDev/include/pcl
    sudo rm -Rf /opt/Inuitive/InuDev/opencv2

    # install patch 
    #sudo rm -Rf ./inutmp
    #sudo mkdir ./inutmp

    #sudo tar -jxvf  $CURRENT_DIR/patch.tar.bz2 --strip-components=1 -C ./inutmp

    #echo -e "\033[32m update firmware for R132 !\033[0m"
    
    # copy nu4000c0.zip patch
    #sudo zip -u /opt/Inuitive/InuDev/bin/NU4000c0/boot300/nu4000c0.zip  $CURRENT_DIR/inutmp/inu_target.out

    # change original point cloud scale to mm
    sudo sed -i "s/<Scale>1.0</<Scale>0.001</g"  $DIST_DIR/../config/InuServiceParams.xml

    #sudo rm -Rf ./inutmp
    
    sudo systemctl start inuservice.service
    sudo systemctl enable inuservice.service

    echo  -e "\033[32m single camera SDK is installed! \033[0m" 
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

installPatch()
{
    # # stop single service
    sleep 1s
    sudo systemctl stop inuservice.service
    sudo systemctl disable inuservice.service

    # #apply patch
    # sleep 1s

    # sudo rm -Rf ./inutmp
    # sudo mkdir ./inutmp

    # sudo tar -jxvf  $CURRENT_DIR/patch.tar.bz2 --strip-components=1 -C ./inutmp

    # echo -e "\033[32m copy all patch files ! \033[0m"
    # sudo cp -av $CURRENT_DIR/inutmp/linux_gcc-*_x86-64/release/*  $DIST_DIR
    
    # sleep 1s

    # copy nu4000c0.zip patch
    # sudo zip -u /opt/Inuitive/InuDev/bin/NU4000c0/boot300/nu4000c0.zip  $CURRENT_DIR/inutmp/inu_target.out

    # sudo rm -Rf ./inutmp


    # copy multi service
    sudo sed -i 's/\r//g' $CURRENT_DIR/multicamera/inuservicemulti.sh
    sudo cp $CURRENT_DIR/multicamera/*  $DIST_DIR
    sudo chmod +x $DIST_DIR/inuservicemulti.sh

    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    
    echo  -e "\033[31m Please input camera number! \033[0m" 
        
    read  -p "camera number(Max=5):"  cameraNum  

    totalcameraNum=$cameraNum

    echo  -e "Bind the camera with USB port: ( No - 0  Yes - 1 )" 

    read  -p "bind the USB port:" bindport

    IAF_VERSION_1=$(grep -w "IAF_VERSION_1" $DIST_DIR/../include/Version.h | awk '{print $NF}' | head -n 1)
    IAF_VERSION_2=$(grep -w "IAF_VERSION_2" $DIST_DIR/../include/Version.h | awk '{print $NF}' | head -n 1)  
    
    version1=$(echo $IAF_VERSION_1 | tr -d -c '[:digit:]')
    version2=$(echo $IAF_VERSION_2 | tr -d -c '[:digit:]')
    
    newVersion=false
    if [ "$version1" -gt 4 ] ; then
        newVersion=true
    elif [ "$version1" -eq 4 ] && [ "$version2" -ge 27 ] ; then
        newVersion=true
    fi

    index=0

    while [ $cameraNum -ge  1 ]; do

        # copy InuService  -> InuService?
        sudo cp $DIST_DIR/InuService  $DIST_DIR/InuService$cameraNum

        # copy InuServiceParams.xml -> InuService?Params.xml
        sudo cp $DIST_DIR/../config/InuServiceParams.xml     $DIST_DIR/../config/InuService${cameraNum}Params.xml

        # change InuService?Params.xml   <InuServiceParams>  -> <InuService?Params>
        sudo sed -i "s/InuServiceParams/InuService${cameraNum}Params/g"  $DIST_DIR/../config/InuService${cameraNum}Params.xml  

        if [ $bindport == 1 ] ; then
            echo "bind the camera to port:  "${filterIds[index]}
            # change  InuService?Params.xml  <FilterId>-1<FilterId>
            if $newVersion; then
                sudo sed -i "s/FilterId>-1</FilterId>${filterIds[index]}</g"  $DIST_DIR/../config/InuService${cameraNum}Params.xml
            else
                sudo sed -i "s/FilterId>-1</FilterId>$(($((${busarray[index]}*1000))+${portarray[index]}))</g" $DIST_DIR/../config/InuService${cameraNum}Params.xml
            fi
        fi
        index=$(($index+1))
        cameraNum=$(($cameraNum-1))
    done

    # cp inuservice@service  -> /lib/systemd/system
    sudo cp $DIST_DIR/inuservice@.service /lib/systemd/system

    sudo systemctl daemon-reload

    sleep 1s

    cameraNum=$totalcameraNum 
    index=1

    while [ $cameraNum -ge  1 ]; do
        sudo systemctl start inuservice@$index.service
        sudo systemctl enable inuservice@$index.service
        index=$(($index+1))
        cameraNum=$(($cameraNum-1))
    done
}

installmultiSDK()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m start to install SDK! \033[0m" 

    removeSDK
    installsingleSDK
    installPatch
    
    echo  -e "\033[31m SDK is installed! \033[0m" 
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

installROS1()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m make sure ROS1 has already installed! \033[0m"
    echo  -e "\033[31m if installed press Enter!  \033[0m"
    echo  -e "\033[31m if not, Ctrl+C to exit! \033[0m"

    read  -p " " anykey
    echo  -e "\033[31m start to install ROS1! \033[0m" 

    if [ $version == "18.04" ]; then
        rosversion="melodic" 
    elif [ $version == "20.04" ]; then 
        rosversion="noetic" 
    elif [ $version == "16.04" ]; then 
        rosversion="kinetic" 
    fi

    sudo dpkg -r ros-$rosversion-inudev-ros-nodelet

    # install ros deb
    sudo dpkg -i $CURRENT_DIR/ros-*.deb

    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m if SDK is installed in sigle-module, just iput 1;\n if SDK is installed in multi-module,make sure the number is same as before! \033[0m" 
    echo  -e "\033[31m Please input camera number! \033[0m" 

    read  -p "camera number(Max = 5):"  cameraNum  

    for ((index = 2; index<6; index++)); do

        seekname="name="\"camera$index\"" default=\".*\""
        if [ $index -le $cameraNum ]; then
            replacename="name="\"camera$index\"" default=\"1\""
        else
            replacename="name="\"camera$index\"" default=\"0\""
        fi
        
        sudo sed -i "s/$seekname/$replacename/g"  /opt/ros/$rosversion/share/inudev_ros_nodelet/launch/inudev_ros_nodelet_multiple.launch
    done

    echo  -e "\033[31m ROS1 is installed! \033[0m" 
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

removeROS()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m remove the inu ros! \033[0m"
    if [ $version == "18.04" ]; then
        rosversion="melodic" 
    elif [ $version == "20.04" ]; then 
        rosversion="noetic" 
    elif [ $version == "16.04" ]; then 
        rosversion="kinetic" 
    fi

    sudo dpkg -r ros-$rosversion-inudev-ros-nodelet

    sudo rm -Rf /opt/ros/$rosversion/share/inudev_ros_nodelet
    sudo rm -Rf /opt/ros/$rosversion/lib/libInuCommonUtilities.so
    sudo rm -Rf /opt/ros/$rosversion/lib/libInuStreams.so
    sudo rm -Rf /opt/ros/$rosversion/lib/libinudev_ros_nodelet.so    

    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

runROS1()
{
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
    echo  -e "\033[31m Start to run ROS1! \033[0m" 

    if [ $version == "18.04" ]; then
        rosversion="melodic" 
    elif [ $version == "20.04" ]; then 
        rosversion="noetic" 
    elif [ $version == "16.04" ]; then 
        rosversion="kinetic" 
    fi

    source /opt/ros/$rosversion/setup.bash
    roslaunch /opt/ros/$rosversion/share/inudev_ros_nodelet/launch/inudev_ros_nodelet_multiple.launch 2>&1 &
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

restartserv()
{
    service=`ps -ef |grep ./InuService|awk -F '/' '{print $2}'`
    servicearray=($service)

    for ((i=0;i<${#servicearray[@]};i++))  do
        result=$(echo ${servicearray[i]}|grep InuService)
        if [[ "$result" != "" ]];then
            tmp=`echo ${servicearray[i]}|awk -F'InuService' '{print $2}'`
            if [[ "$tmp" != "" ]]; then
              sudo systemctl stop inuservice@$tmp.service
              sleep 1s
              echo "systemctl start inuservice@$tmp.service"
              sudo systemctl start inuservice@$tmp.service

            else
              sudo systemctl stop inuservice.service
              sleep 1s
              echo "systemctl start inuservice.service"
              sudo systemctl start inuservice.service
            fi    
        fi
    done
    echo  -e "\033[32m @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \033[0m"
}

banner
checkCamera

while :
do
    Usage
    read  -p "Input the selection:"  num

    if [ -z $num ]; then
        continue
    fi

    if [ $num = 'f' ]; then
        checkCamera
        read  -p "press any key to continue! " anykey


    elif [ $num = 1 ]; then
        removeSDK
        installsingleSDK 
        removeROS
    elif [ $num = 2 ]; then  
        installmultiSDK
        removeROS
    # elif [ $num = 3 ]; then
    #     installROS1
    # elif [ $num = 5 ]; then
    #     runROS1
    elif [ $num = 6 ]; then
        restartserv    
    elif [ $num = 's' ]; then
        setConfidence
    elif [ $num = 'u' ]; then
        removeSDK
    elif [ $num = 'q' ]; then
        echo "quit!" 
        exit 1
    fi
done
