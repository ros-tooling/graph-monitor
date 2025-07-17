# Message representing parameters for a single ROS2 node

string node_name  # Fully qualified node name (e.g., "/my_node")
string node_namespace  # Node namespace (e.g., "/")
Parameter[] parameters  # Array of parameters for this node
