# topic_stats_collector

Publishes the topic statistics that every instrumented process on this machine has written to shared
memory. One of these per machine.

It is an ordinary node with no special privileges, and nothing it does affects the processes it
reads from: they write into their own segments whether or not this is running, and they do not
notice if it stops.

## Running

```shell
ros2 launch topic_stats_collector collector.launch.py
```

Parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `poll_period_ms` | `200` | How often to drain every segment. Must be fast enough that a writer does not lap its ring between polls. |
| `output_topic` | `/topic_statistics` | Where to republish. |
| `reclaim_dead_segments` | `true` | Remove segments left behind by processes that have exited. |
| `skip_backlog_on_attach` | `true` | Discard what a newly discovered writer already accumulated. |

The processes being measured need `ROS_TOPIC_STATISTICS_EGRESS=shared_memory`; see the
`rmw_stats_shim` README.

If the stack runs across containers, this collector and every process it measures must share
`/dev/shm`, which means `--ipc=host` or an equivalent. One collector serves every container sharing
that `/dev/shm`, so it is one per host rather than one per container. It does not need to share a
PID namespace with anything: writer liveness comes from an advisory lock, not from pids.

## Why the pool is separate from the node

`SegmentPool` holds everything hard about this: writers appearing, writers exiting, segments left
behind by crashes, pids being reused, segments from a build that disagrees about the format. None of
that is about ROS, and all of it is worth testing without a middleware, so it lives in its own class
with process liveness injected.

`skip_backlog_on_attach` deserves a word. A process that has been running for hours holds a ring
full of history whose timestamps are long past. Replaying that on attach would look like a burst of
very stale traffic to whatever consumes the topic, so by default a newly discovered writer is
fast-forwarded to its current head. The cost is that a writer which starts in the gap between two
polls may have its first snapshot skipped.

## What it reports about itself

Lost samples mean this collector is not keeping up, which downstream looks like topics that
intermittently appear stale. It logs that, rate limited, rather than letting it pass silently. If it
happens, poll faster or raise `ROS_TOPIC_STATISTICS_SHM_CAPACITY` on the writers.
