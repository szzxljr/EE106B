#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_pykdl"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/install/lib/python2.7/dist-packages:/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/build" \
    "/usr/bin/python" \
    "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_pykdl/setup.py" \
    build --build-base "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/build/baxter_pykdl" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/install" --install-scripts="/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/install/bin"
