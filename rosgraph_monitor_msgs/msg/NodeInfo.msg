# Fully qualified node name
string name

Topic[] publishers
Topic[] subscriptions

# Observed parameter descriptions and current values for this node
# parameter_values[] must be the same size as parameters[], or empty
# Empty on either array means no values have yet been successfully queried
# When set, parameter_values[] match up 1:1 with the same index in parameters[]
rcl_interfaces/ParameterDescriptor[] parameters
rcl_interfaces/ParameterValue[] parameter_values
