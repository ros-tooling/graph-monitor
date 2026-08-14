# Developing graph-monitor

## Code Standard

Instead of the `ament_lint` tooling, this repository uses [pre-commit](https://pre-commit.com) hooks from [polymath_code_standard](https://github.com/polymathrobotics/polymath_code_standard)

One-time setup in the repo root:

```shell
pre-commit install
```

Now, all formatting and linting happens automatically on commit, instead of at test time.
No more manual formatting!
