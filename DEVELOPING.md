# Developing graph-monitor

## Code Standard

Instead of the `ament_lint` tooling, this repository uses [pre-commit](https://pre-commit.com) hooks from [polymath_code_standard](https://github.com/polymathrobotics/polymath_code_standard)

One-time setup in the repo root:

```shell
pre-commit install
```

Now, all formatting and linting happens automatically on commit, instead of at test time.
No more manual formatting!

## Updating/Refreshing parameters.md

When the `params_decl.yaml` changes, the documentation page for the parameters in `doc/parameters.md` needs to be updated. To do that, we use `generate_parameter_library`. Follow the steps below to update the `doc/parameters.md`.

Source your ROS installation then install `generate-parameter-library` tool.

```shell
sudo apt install ros-$ROS_DISTRO-generate-parameter-library
```

Run the following command from the `graph-monitor/` directory or modify the input and output path as necessary.

```shell
generate_parameter_library_markdown --output_markdown_file rosgraph_monitor/doc/parameters.md --input_yaml_file rosgraph_monitor/src/params_decl.yaml
```
