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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/bargos/offboard/src/rospy-template"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/bargos/offboard/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/bargos/offboard/install/lib/python2.7/dist-packages:/home/bargos/offboard/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bargos/offboard/build" \
    "/usr/bin/python2" \
    "/home/bargos/offboard/src/rospy-template/setup.py" \
     \
    build --build-base "/home/bargos/offboard/build/rospy-template" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/bargos/offboard/install" --install-scripts="/home/bargos/offboard/install/bin"
