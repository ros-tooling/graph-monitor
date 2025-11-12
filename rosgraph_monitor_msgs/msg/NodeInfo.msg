# Fully qualified node name (FQN)
string name

Topic[] publishers
Topic[] subscriptions

# TODO:
# - Services
# - Actions

# Observed parameter descriptions this node
rcl_interfaces/ParameterDescriptor[] parameters

# Observed current parameter values for this node
# parameter_values[] must be the same size as parameters[], or empty
# Empty on either array means no values have yet been successfully queried
# When set, parameter_values[] match up 1:1 with the same index in parameters[]
# NOTE: parameter_values[] is not yet implemented, so will always be empty
rcl_interfaces/ParameterValue[] parameter_values
