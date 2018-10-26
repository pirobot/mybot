#!/bin/bash

function usage()
{
    echo "Invalid command. Valid arguments are:"
    echo ""
    echo "--help"
    echo "--output-name=<filename>"
    echo "--output-prefix=<filename-prefix>"
    echo "--duration=<seconds>(m|h)"
    echo "--size=<mb>"
    echo "--split"
    echo ""
}

ARGS=""

while [ "$1" != "" ]; do
    PARAM=`echo $1 | awk -F= '{print $1}'`
    VALUE=`echo $1 | awk -F= '{print $2}'`
    case $PARAM in
        --help)
            usage
            exit
            ;;
        --output-prefix|--duration|--size)
        	if [ ! -z "$VALUE" ]
        	then
            	ARGS="$ARGS $PARAM=$VALUE"
            fi
            ;;
        --split)
            ARGS="$ARGS $PARAM"
            ;;
        --output-name)
        	if [ ! -z "$VALUE" ]
        	then
        		EXPANDED_FILENAME="${VALUE/\~/$HOME}"
       			if ! [[ "$EXPANDED_FILENAME" =~ ^/.* ]]
            	then
            		echo "`pwd`"
            		EXPANDED_FILENAME="`pwd`/$EXPANDED_FILENAME"
            	fi
            	echo "$EXPANDED_FILENAME"
            	ARGS="$ARGS $PARAM=$EXPANDED_FILENAME"
            fi
            ;;
        __name:|__log:)
            ;;
        *)
            echo "ERROR: unknown parameter \"$PARAM\""
            usage
            exit 1
            ;;
    esac
    shift
done

if [ -z "$ARGS" ]
then
	ARGS="--output-prefix=rosbag_test"
fi

rosbag record -b 512 --lz4 $ARGS \
arduino/sensor/ir_front_center \
arduino/sensor/sonar_front_left \
arduino/sensor/sonar_front_center \
arduino/sensor/sonar_front_right \
arduino/sensor/sonar_rear_left \
arduino/sensor/sonar_rear_center \
arduino/sensor/sonar_rear_right \
arduino/sensor_state \
cmd_vel_mux/active \
joy \
joy_deadman \
odom \
rosout \
rosout_agg \
cmd_vel \
cmd_vel_mux/active \
cmd_vel_mux/input/ar_tag \
cmd_vel_mux/input/navi \
cmd_vel_mux/input/teleop \
cmd_vel_mux/input/twist \
cmd_vel_mux/input/safety_controller \
diagnostics \
diagnostics_agg \
start_stop_trajectory \
tf \
tf_static \
waiting_for_person

