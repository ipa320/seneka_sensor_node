#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/catkin_generated', type 'exit' to leave"
  . "/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/catkin_generated'"
else
  . "/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
