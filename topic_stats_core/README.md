# topic_stats_core

Rolling-window statistics for ROS topic endpoints, with no ROS dependencies.

This is the middle layer of a three-part design. It knows how to turn observed message events into
statistics, and nothing else. It does not know how those events were observed, and it does not know
where the resulting statistics go.

```text
   ingest adapter          topic_stats_core            egress adapter
  ----------------        ------------------          ----------------
  RMW wrapper shim  --->  Recorder                    StatsSink  --->  ROS topic
  tracetools preload -->    Collector    --snapshot-->             --->  shared memory
  offline replay    --->  StatsReport                              --->  ...
```

## Why the layers are split this way

Two ingest mechanisms are in play. One intercepts RMW calls, which requires a patched
`rmw_implementation` to load a wrapper library. The other interposes the `ros_trace_*` functions
that `rcl` and the RMW implementations already call, which requires nothing of anybody. Both observe
the same events, so the statistics code should not care which one is running, and it should be
possible to run both at once and diff them.

Egress is similarly unsettled. Publishing on a ROS topic from inside the middleware's own call stack
works but is delicate; a shared-memory handoff to a separate collector process avoids that entirely.
Sinks receive plain structs so that the choice stays swappable and the ROS message contract lives in
exactly one place.

Keeping this package free of ROS dependencies is what makes it testable without a middleware, a
node, a launch file, or a tracing session.

## The two boundaries

`Recorder` (`recorder.hpp`) is everything an ingest adapter may do: register nodes and endpoints,
and report that a message was published or taken. Adapters translate their native events into these
calls and remember the mapping from their own handles to the returned ids. They do not compute
statistics, do not sample time, and never touch a transport.

`StatsSink` (`sink.hpp`) is everything an egress adapter may do: accept a `StatsReport`.

## Design notes worth knowing before changing this

**Two clocks, not one.** Periods are measured on a monotonic clock so that an NTP step cannot
manufacture a huge or negative interval. Take age has to be measured against the publisher's
system-clock source timestamp, so the collector needs both. `Clock` is injected, which is what makes
the tests deterministic and is also the seam an offline trace replay would drive.

**Generation-checked handles.** Endpoint slots are recycled, because a process that churns
subscriptions would otherwise grow a slot table without bound. Recycling means a stale handle could
silently record onto whatever endpoint took the slot next, so ids carry a generation counter and
stale records are counted in `diagnostics()` instead.

**Nodes may be unregistered before their endpoints.** RMW teardown does not guarantee otherwise, so
`unregister_node` drops the node's endpoints itself.

**Absent measurements are not zero measurements.** An endpoint that has never been measured is
omitted from reports entirely, even in `SnapshotMode::All`. Reporting a zero would be
indistinguishable downstream from a real measurement.

**`OnlyChanged` snapshots consume.** An endpoint that has gone quiet stops appearing in reports,
which is what lets downstream staleness detection notice.

**Locking.** A shared mutex guards the registry structure; each endpoint's measurements are guarded
individually. The hot path is one uncontended shared lock plus one uncontended mutex, and recording
on unrelated endpoints does not contend. Lock order is always registry then endpoint.

## Testing

```shell
colcon build --packages-select topic_stats_core
colcon test --packages-select topic_stats_core
```

The concurrency tests are worth much more under ThreadSanitizer, and the repository's mixins already
provide it:

```shell
colcon build --mixin tsan --packages-select topic_stats_core
colcon test --packages-select topic_stats_core
```

`test_collector_concurrency.cpp` exercises recording against concurrent registration, endpoint
churn that recycles slots underneath live handles, and node teardown that orphans endpoints. Those
cases are there because the predecessor of this code mutated unsynchronized maps from arbitrary
publisher threads while a timer thread iterated them, and got away with it only because endpoints
are almost always created at startup.
