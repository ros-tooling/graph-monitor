# topic_stats_shm

Shared memory transport for topic statistics. An instrumented process writes its snapshots into its
own segment; a separate collector reads every segment on the machine.

This is the "E2" egress. Its point is that an instrumented process never creates a ROS entity, never
links the middleware to report, and never appears in the graph it is measuring. Nothing in this
package can re-enter the middleware, so unlike the publishing egress it is safe to call from inside
a middleware call stack.

Has no ROS dependencies. `topic_stats_collector` is what turns what this produces into messages.

## Containers

Writers and the collector must share `/dev/shm`. Docker gives each container a private one by
default, so this needs `--ipc=host` (which also shares the host's `/dev/shm`),
`--ipc=container:<name>`, or an explicitly shared mount. Sharing IPC does **not** share the PID
namespace, which is why nothing here identifies a process by its pid alone.

## Layout

One segment per writing process, named `topic_stats.<pid namespace>.<pid>`, so a collector can
enumerate every producer and tell which process in which container each belongs to without opening
it. The PID namespace's inode is allocated host-wide and is unique per namespace; a bare pid is not,
because every namespace numbers from 1 and two containers running a few nodes each will collide in
the low tens almost immediately.

A segment is a 256 byte header followed by a ring of 256 byte fixed-size samples. Fixed size because
slots have to be addressable by index; 256 bytes because that leaves room for a node name and a
topic name inline, which avoids any pointer chasing across the boundary.

A ring rather than a single latched snapshot, because the statistics core *consumes* measurements
when it snapshots. A reader that falls behind a latch would lose them permanently instead of merely
reading them late. With a ring, falling behind costs you only what the writer laps, and that loss is
counted and reported rather than silent.

## How tearing is prevented

Single writer, many readers, no locks. Every slot carries a sequence number, which the writer clears
before touching the payload and sets to `index + 1` after. A reader checks it before and after
copying the payload; if it changed, the writer lapped the reader mid-copy and the sample is
discarded and counted as `torn`.

This makes cross-process atomics load bearing, so the header static_asserts that they are lock free.
It also makes the layout a contract between two independently built processes, so the header carries
a format version, the record size, and the header size. A reader that does not recognise all three
refuses the segment rather than guessing at it.

## How liveness works

A writer holds an advisory lock (`flock`) on its segment for its entire lifetime. A reader decides
the writer is gone by successfully taking that lock, and immediately dropping it again.

This is deliberately not a question about pids. A pid from another PID namespace is either absent
from the reader's namespace, which would make a healthy writer look dead and get its segment
reclaimed out from under it, or it collides with an unrelated local process, which would make a dead
writer look alive forever. The lock has neither problem: the kernel releases it on process death
however abrupt, it belongs to the open file rather than to a process number, and a reader can probe
it on a read-only descriptor.

Creating a segment takes the same lock, and refuses if it is already held. Under a shared
`/dev/shm`, a name collision is a cross-container accident rather than an impossibility, and
replacing a running writer's segment would silently destroy its statistics.

## Failure modes it takes seriously

- **A writer crashes.** Its segment outlives it, but its lock does not, so readers can tell and
  reclaim it. That matters because `/dev/shm` is shared with the DDS implementation's own segments
  and with every other container.
- **A segment is reused.** Creation truncates to zero before resizing, so stale contents never
  survive, and a reader whose write counter goes backwards resyncs instead of reporting a backlog of
  billions.
- **A collector is too slow.** Samples are overwritten. This is counted as `lapped` and surfaced, so
  it looks like what it is rather than like a quiet system.
- **Names are too long.** Truncated, flagged, and counted. A shortened name still identifies the
  problem; a dropped sample does not.
- **A segment cannot be created at all.** The sink reports the failure and the caller carries on
  without statistics. Instrumentation must never stop a process from starting.

## Testing

```shell
colcon build --packages-select topic_stats_shm
colcon test --packages-select topic_stats_shm
```

`test_round_trip.cpp` includes a reader draining while a writer laps it continuously, asserting that
every sample that comes out is internally consistent. That one is worth running under the
sanitizer, which CI does:

```shell
colcon build --mixin tsan --packages-up-to topic_stats_shm
colcon test --packages-select topic_stats_shm
```
