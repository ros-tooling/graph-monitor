# Statistics about one measurement for a single endpoint of a topic

uint8 PUBLISHED_PERIOD=0
uint8 RECEIVED_PERIOD=1  # inverse of frequency
uint8 TAKE_AGE=2  # the system time difference between DDS publish and rmw_take
uint8 statistic_type  # one of above constants

string node_name  # name of the node holding the endpoint (publisher or subscription)
string topic_name  # topic being measured
int32 window_count  # how many messages were in the calculation

builtin_interfaces/Duration mean
builtin_interfaces/Duration min
builtin_interfaces/Duration max
