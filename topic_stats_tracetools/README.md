# topic_stats_tracetools

Topic statistics gathered from the instrumentation that `rcl` and the RMW implementations already
call, with no patched `rmw_implementation` and no changes to any node.

This is the alternative to `rmw_stats_shim`. That one needs a fork of `rmw_implementation` to load a
wrapper library, which is a hard thing to ask upstream to carry. This one needs nothing from anyone:
the `ros_trace_*` functions are already there, already called, and already enabled in distribution
binaries.

## Using it

```shell
LD_PRELOAD=libtopic_stats_tracetools_preload.so ros2 run your_package your_node
```

Then run `topic_stats_collector` somewhere on the machine to turn what it writes into
`/topic_statistics`.

| Variable | Default | Meaning |
|---|---|---|
| `ROS_TOPIC_STATISTICS_WINDOW_SIZE` | `50` | Messages per rolling window |
| `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` | `1.0` | Seconds between reports |
| `ROS_TOPIC_STATISTICS_EGRESS` | `shared_memory` | `shared_memory` or `none` |
| `ROS_TOPIC_STATISTICS_SHM_CAPACITY` | `256` | Ring slots in the segment |
| `ROS_TOPIC_STATISTICS_IDLE_EVICTION_REPORTS` | `600` | Quiet reports before an endpoint is dropped |

Shared memory is the only egress offered here. Publishing on a ROS topic would mean creating an RMW
publisher from inside a tracepoint that the middleware calls during `rmw_publish`, which is the
reentrancy problem at its sharpest.

## How the interposition works

ELF symbol resolution takes the first definition in the global lookup scope, and an `LD_PRELOAD`ed
library comes before the `DT_NEEDED` libraries of everything else. So the `ros_trace_*` definitions
in `interpose.cpp` win over the real `libtracetools` for the symbols named there, and **only** those
symbols; everything else still resolves to the real library.

Each one then chains to the real implementation through `RTLD_NEXT`, so a live LTTng session records
exactly what it would have recorded without this library. Preloading this does not turn tracing off,
and does not require tracing to be on.

Three variants are defined per tracepoint because callers use two forms. `rcl` and most RMW
implementations call `ros_trace_X` directly, but `rmw_zenoh_cpp` uses the guarded pair
`ros_trace_enabled_X` then `ros_trace_do_X`. Interposing only the plain form would silently miss
every zenoh event.

## The join

- `rcl_node_init` names a node
- `rcl_publisher_init` / `rcl_subscription_init` give the topic and the **rmw** handle
- `rmw_publish` / `rmw_take` carry that rmw handle, and takes carry the source timestamp

Keyed on rmw handles rather than rcl handles deliberately. The `rcl_publisher_t` passed to
`rcl_publisher_init` can be a stack temporary, which was observed for `/rosout`, so it is not a
stable identity. The rmw handles are heap allocated and live as long as the endpoint.

## What it cannot see, and what that costs

**There are no destruction tracepoints.** ros2_tracing has `rcl_publisher_init` and no
`rcl_publisher_fini`, for any entity. Nothing ever says an endpoint went away, which has two
consequences.

The registry would grow for the life of the process, so endpoints that report nothing for
`ROS_TOPIC_STATISTICS_IDLE_EVICTION_REPORTS` consecutive reports are dropped. That is only safe
because this adapter keeps the endpoint's descriptor: a topic that publishes once an hour is
evicted and then registered afresh when it finally speaks, rather than being lost for the life of
the process. `revived_endpoints()` counts how often that happens, and a nonzero value means the
threshold is too aggressive for the traffic here.

The middleware is also free to hand back a freed handle's address for a new endpoint. A repeated
init on a known handle therefore replaces the mapping and unregisters the old endpoint, rather than
leaving the new topic's traffic attributed to the old one.

**Intra-process publishes are invisible.** They never reach `rmw_publish`; they go through
`rclcpp_intra_publish` and the ring buffer tracepoints. The RMW wrapper has exactly the same blind
spot, but here it is fixable, since those tracepoints exist and could be interposed too.

**Unattributed rmw handles are normal.** The middleware has publishers of its own for discovery that
`rcl` never initialises, and their tracepoints fire before any node exists. Those are ignored.

## Testing

```shell
colcon build --packages-select topic_stats_tracetools
colcon test --packages-select topic_stats_tracetools
```

The tests drive `TraceCollector` directly with fabricated handles, so they need no middleware, no
node, and no tracing session. That is the point of keeping `interpose.cpp` free of logic: it is the
part that cannot be unit tested, so there is nothing in it worth testing.
