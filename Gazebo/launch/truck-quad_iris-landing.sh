#!/bin/bash

# Truck - Quad Iris Landing Example
# copied from truck-quadplane-landing

# Modifications
# Replaced quadplane with quad_iris
# Replaced arduplane with arducopter
# Replaced PLANE with COPTER
# Replaced Quadplane with Quad_iris
# Replaced plane with copter

# Kill all SITL binaries when exiting
trap "killall -9 arducopter & killall -9 ardurover" SIGINT SIGTERM EXIT

# assume we start the script from the root directory (ARDUPILOT_HOME)
ROOTDIR=$ARDUPILOT_HOME
COPTER=$ROOTDIR/build/sitl/bin/arducopter
ROVER=$ROOTDIR/build/sitl/bin/ardurover

# edit the location of the SITL_Models directory if different
SITL_MODELS_DIR="$ARDUPILOT_HOME/../simulation/SITL_Models"

# When used with Gazebo - ArduPilot-SITL, the home location
# is used to set the world origin - so this should be the same
# for all vehicles. Vehicle positions within the world are determined
# from the initial poses specified in truck_quad_iris_landing.sdf
HOMELAT=-35.363262
HOMELONG=149.165237
HOMEALT=584.0

#--------------------------------------------------------------------
# Gazebo world generation

mkdir -p sitl/worlds

# generate the world
pushd $SITL_MODELS_DIR/Gazebo/worlds
erb -T 1 truck_quad_iris_landing.sdf.erb > $ROOTDIR/sitl/worlds/truck_quad_iris_landing.sdf
popd

#--------------------------------------------------------------------
# ArduPilot SITL

# build binary if not present
[ -x "$COPTER" -a -x "$ROVER" ] || {
    ./waf configure --board sitl --debug
    ./waf copter rover
}

#--------------------------------------------------------------------
# Truck - tractor

mkdir -p sitl/tractor/scripts

# copy the mixer script to sitl/tractor/scripts
cp -f "$SITL_MODELS_DIR/Gazebo/scripts/daf_xf_450_tractor_mixer.lua" sitl/tractor/scripts

TRACTOR_DEFAULTS="$SITL_MODELS_DIR/Gazebo/config/daf_xf_450_tractor.param"

# additional parameter file for the tractor unit
cat <<EOF > sitl/tractor/leader.param
SYSID_THISMAV 1
WP_RADIUS 5.0
WP_SPEED 5.0
EOF

(cd sitl/tractor && $ROVER -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --instance 0 --defaults $TRACTOR_DEFAULTS,leader.param) &

#--------------------------------------------------------------------
# Truck - trailer

mkdir -p sitl/trailer

TRAILER_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/rover.parm"

# additional parameter file for the trailer unit
cat <<EOF > sitl/trailer/leader.param
SYSID_THISMAV 2
EOF

(cd sitl/trailer && $ROVER -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --instance 1 --defaults $TRAILER_DEFAULTS,leader.param) &

#--------------------------------------------------------------------
# Copter 1

COPTER1_NAME=iris

mkdir -p sitl/$COPTER1_NAME

COPTER_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/gazebo-iris.parm"

# additional parameter file for the iris
cat <<EOF > sitl/$COPTER1_NAME/follower.param
SYSID_THISMAV 3
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_Y 0
FOLL_OFS_Z 10
FOLL_OFS_TYPE 1
FOLL_SYSID 2
FOLL_DIST_MAX 1000
FOLL_YAW_BEHAVE 2
FOLL_ALT_TYPE 1
EOF

(cd sitl/$COPTER1_NAME && $COPTER -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --instance 2 --defaults $COPTER_DEFAULTS,follower.param) &

wait

