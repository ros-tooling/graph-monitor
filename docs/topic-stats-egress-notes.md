<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
SPDX-License-Identifier: Apache-2.0
-->

# Egress prototypes: assumptions, decisions, and open questions

Written alongside the E1 and E2 egress prototypes so that the judgement calls made without you are
visible rather than buried in the diff. Nothing here is settled; it is what to argue with.

## What was built

Both egress paths are complete and selected per process by `ROS_TOPIC_STATISTICS_EGRESS`:

- **E1, `rmw_publisher`** (default, pre-existing): a `/topic_statistics` publisher per node, created
  directly against the RMW implementation.
- **E2, `shared_memory`**: a per-process segment under `/dev/shm`, drained by a separate
  `topic_stats_collector` node that republishes on the same topic.
- **`none`**: measure without reporting, for isolating ingest cost.

The swap is proven by `topic_stats_collector/test/test_egress_equivalence.cpp`, which drives one real
`Collector`, sends the identical snapshot down both paths, and requires identical `TopicStatistics`
out. It pins the expected statistic count so it cannot pass by trivially matching one sample.

## Decisions made without asking

**Package split.** E1 lives in `rmw_stats_shim` and E2 in `topic_stats_shm`, rather than both in a
shared "sinks" package. E1 is not actually a generic sink: creating a publisher needs an
`rmw_node_t` and the implementation library's symbols, which only the RMW ingest adapter has. That
asymmetry is real and worth seeing rather than hiding behind a common interface, and it is itself an
argument for E2. `StatCollector` holds a non-owning `RmwPublisherSink*` alongside the owning
`StatsSink`, non-null only when E1 is selected, to reach the per-node setup the generic interface
does not expose.

**`topic_stats_ros` is a separate package** holding only the snapshot to message conversion. Both
egress paths must produce identical messages, so there should be exactly one copy of that code, and
it must not drag `rclcpp` into every instrumented process. `topic_stats_collector` is separate again
because it does need `rclcpp`.

**E2 is a ring, not a latched latest snapshot.** The core *consumes* measurements when it snapshots,
so a reader that falls behind a latch would lose them permanently rather than reading them late.
With a ring, loss is bounded by the ring size, and is counted and reported rather than silent.

**Segments are named `topic_stats.<pid namespace>.<pid>`, and liveness comes from an advisory
lock, not from a pid.** Both because of containers; see the section below.

**Backlog is skipped when the collector attaches to a writer.** A process running for hours holds a
ring of history whose timestamps are long past, and replaying it would look like a burst of stale
traffic. The cost: a writer that starts and reports between two polls may have its first snapshot
skipped. Configurable via `skip_backlog_on_attach`.

**Failures are non-fatal everywhere.** If a segment cannot be created, the process logs and runs
without statistics. If the egress name is unrecognised it falls back to the default rather than to
`none`, because reporting nothing looks exactly like a healthy silent system.

**Losses are counted, never swallowed.** `lapped`, `torn`, `unknown_statistics`, `rejected`,
`unrepresentable_samples`, `truncated_names`. The collector logs them rate limited.

## Containers

The stack runs distributed across containers on one host, with `--ipc=host`. That makes `/dev/shm`
shared, which is the easy half. The hard half is that `--ipc=host` does **not** share the PID
namespace, and the first version of this keyed two things on `getpid()`.

**Names collided.** Every PID namespace numbers from 1, so two containers each running a few nodes
collide in the low tens almost immediately. Creating a segment used to replace any existing one of
the same name, so one container's writer would have silently destroyed another's.

**Liveness was worse than wrong.** It asked `kill(pid, 0)` in the collector's namespace. A pid from
another container either does not exist locally, in which case the collector concludes a healthy
writer is dead, unlinks its segment, and that process's statistics stop permanently and silently; or
it happens to match an unrelated local process, in which case a dead writer looks alive forever and
its segment leaks.

Both are fixed:

- Identity is now `(PID namespace inode, pid)`. The namespace inode is allocated host-wide and is
  unique per namespace, so the pair is unambiguous wherever `/dev/shm` is shared, and it still says
  which process in which container a segment came from.
- Liveness is now an **advisory lock** (`flock`) that the writer holds on its segment for its
  lifetime. The kernel releases it on process death however abrupt, it is a property of the open
  file rather than of a pid, and a reader can probe it on a read-only descriptor. Namespaces do not
  enter into it.
- Creating a segment now refuses to take one whose lock is held, rather than replacing it. Under
  `--ipc=host` a name clash is a cross-container accident, not an impossibility.

Note that a collector serves every container sharing the `/dev/shm` it can see, so with `--ipc=host`
that is one collector for the whole host, not one per container.

## Assumptions baked into E2

These are the ones most likely to be wrong for your deployment.

1. **Linux, and `/dev/shm` specifically.** Enumerating POSIX shared memory objects is not portable,
   so `Segment::list` reads the tmpfs directly. Everything else uses `shm_open`.
2. **`/dev/shm` is shared between the writers and the collector.** True under your `--ipc=host`
   default. It would also work with `--ipc=container:<name>` or a shared mount, and it does not work
   at all with Docker's default private per-container `/dev/shm`.
3. **All writers and the collector run as the same user.** Segments are created `0600`. A collector
   running as a different user silently sees nothing. This matters more across containers than
   within one, and whether it should be `0640` with a shared group is a deployment question I could
   not answer.
4. **Same host.** There is no cross-host story; a multi-machine graph needs one collector per host,
   all publishing to the same topic.
5. **Writers and the collector are built from the same source.** The header carries a format
   version, record size, and header size, and a mismatch on any of them refuses the segment. That
   turns ABI skew into a visible refusal rather than corruption, but it does mean a rolling upgrade
   drops statistics until both sides are updated. Relevant here because containers are updated
   independently.
6. **Names fit in 80 and 120 bytes.** Node FQNs are capped at 79 characters and topic names at 119,
   after which they are truncated and flagged. Those numbers came from making the record exactly
   256 bytes, not from measuring your actual names.
7. **`/dev/shm` has room.** Default is 64KiB per instrumented process. Under `--ipc=host` this is
   the host's `/dev/shm`, usually far larger than the 64MiB a container gets by default, so this is
   unlikely to bind.

## Open questions

1. **Which one ships?** Closer than I first said. E2's argument stands: the instrumented process
   creates no ROS entity, cannot re-enter the middleware, and does not perturb the graph it
   measures. But E1 works across your containers today with no changes at all, because it publishes
   over DDS, which is already how your stack communicates. E2 needs `--ipc=host` to stay on, one
   collector per host, and everything in the Containers section above. Whether E1's perturbation is
   worth avoiding at that price is a deployment judgement I cannot make from here.

2. **Segment permissions.** `0600` today. Do all your containers run as the same UID? If not this
   has to become `0640` with a shared group, and widening permissions on shared memory
   speculatively seemed wrong.

3. **Is one collector per host acceptable operationally?** It is a new process to supervise, in its
   own container presumably, and it has to keep `--ipc=host`. If the answer is no, E1 stops being a
   stepping stone and becomes the answer.

4. **Should the collector republish or aggregate?** It currently republishes one message per node per
   snapshot, preserving E1's shape exactly. It could instead batch a whole poll into one message,
   which would be cheaper but would change what `rosgraph_monitor` sees.

5. **Poll period versus ring capacity.** Defaults are 200ms and 256 slots against a 1Hz writer, a
   large margin. Neither is tuned; the safe combination depends on how many endpoints your processes
   actually have.

6. **`TAKE_AGE` is still discarded** by `rosgraph_monitor`, in the `else { continue; }` of
   `on_topic_statistics`. Both egress paths carry it faithfully, including negative values from
   clock skew. Nothing consumes it yet.

## Verified by hand, not by a test

The E2 path was exercised across real process boundaries with a standalone writer and the collector
node running together. Confirmed: the collector discovers a writer that appears after it starts,
notices a writer that exits cleanly, and reclaims the segment of a writer killed with SIGKILL
(`/dev/shm` went back to zero segments). None of this is automated.

Not verified: any of it across actual container boundaries. The namespace-qualified naming and the
lock-based liveness are unit tested, including a writer fabricated in a different PID namespace with
pid 1, but nothing has run in two real containers.

## Not done

- No automated integration test running a real node under the shim with `shared_memory` egress and a
  live collector. Every layer below that is unit tested, and the cross-process behaviour above was
  checked by hand, but nothing in CI covers the two together, because that needs the forked
  `rmw_implementation` in the loop.
- Nothing asserts on the collector's published topic. The message content is covered by the
  equivalence test, and the publish call itself is one line, but no test subscribes.
- No benchmark comparing the two paths. Deciding between them on deployment grounds probably wants
  one.
- `pre-commit` is not installed in this container, so the `polymath_code_standard` hooks have not run
  over any of this.
