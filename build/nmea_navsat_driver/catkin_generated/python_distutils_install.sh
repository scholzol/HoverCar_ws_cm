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

echo_and_run cd "/home/ubuntu/HoverCar_ws_cm/src/nmea_navsat_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/HoverCar_ws_cm/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/HoverCar_ws_cm/install/lib/python3/dist-packages:/home/ubuntu/HoverCar_ws_cm/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/HoverCar_ws_cm/build" \
    "/usr/bin/python3" \
    "/home/ubuntu/HoverCar_ws_cm/src/nmea_navsat_driver/setup.py" \
     \
    build --build-base "/home/ubuntu/HoverCar_ws_cm/build/nmea_navsat_driver" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/HoverCar_ws_cm/install" --install-scripts="/home/ubuntu/HoverCar_ws_cm/install/bin"
