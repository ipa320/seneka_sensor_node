#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/devel', type 'exit' to leave"
  . "/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/devel'"
else
  . "/home/cmm-cm/ros_workspace/ros_workspace/sande/seneka/seneka/seneka_dgps/build/devel/setup.sh"
  exec "$@"
fi
