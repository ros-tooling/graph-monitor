# Graph structure representing the ROS graph state. Published on graph change updates.
# TODO:
# - Include publisher + subscriber info (includign qos)
# - Include service info
# - Include action info
# - Include parameter info

builtin_interfaces/Time timestamp
NodeInfo[] nodes
