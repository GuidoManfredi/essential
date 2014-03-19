#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/gmanfred/devel/ros/packs/vision/carod/build/devel', type 'exit' to leave"
  . "/home/gmanfred/devel/ros/packs/vision/carod/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/gmanfred/devel/ros/packs/vision/carod/build/devel'"
else
  . "/home/gmanfred/devel/ros/packs/vision/carod/build/devel/setup.sh"
  exec "$@"
fi
