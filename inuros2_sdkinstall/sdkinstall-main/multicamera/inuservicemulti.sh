#!/bin/sh

start () {
    # Put here the command to start your application
    INU_BIN_RDIR=/opt/Inuitive/InuDev/bin

    export LD_LIBRARY_PATH=$INU_BIN_RDIR:$LD_LIBRARY_PATH

    cd $INU_BIN_RDIR
    ./InuService$1 1>/tmp/$$.inuservice$1.log.txt 2>&1 &
    IS_INUSERVICE_RUN=`ps -e | grep InuService$1 | wc -l | awk '{print $0}'`
    if [ "$IS_INUSERVICE_RUN" = "0" ]
    then
        echo "InuService$1 Starting failed"
    else
        INUSERVICE_PID=`ps -e | grep InuService$1 | awk '{print $1}'`
        echo "InuService$1 Started: "$INUSERVICE_PID
    fi
}

stop () {
    # Put here the command to stop your application
    IS_INUSERVICE_RUN=`ps -e | grep InuService$1 | wc -l | awk '{print $0}'`
    if [ "$IS_INUSERVICE_RUN" -ne "0" ]
    then
        INUSERVICE_PID=`ps -e | grep InuService$1 | awk '{print $1}'`
        kill -9 $INUSERVICE_PID
        echo "InuService$1 stopped"
    fi
}

case "$1" in
start)
        start $2
        ;;
stop)
        stop $2
        ;;
restart)
        stop $2
        sleep 1
        start $2
        ;;
*)
        echo "Usage: $0 { start | stop | restart }"
        exit 1
        ;;
esac

exit 0
