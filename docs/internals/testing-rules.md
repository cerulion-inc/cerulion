# Testing rules

Six rules every test in this repository is written to, and every review thread on a pull
request is answered against. They hold for contributors adding a test and for reviewers
reading one. Each rule states what to do, why it is a rule rather than a preference, and
one place in the tree that already shows the shape.

## 1. Every oracle is derived independently of the code under test

Compare against hand written bytes, a second derivation in another language or tool, or
the artifact the user actually sees; a second run of the same code is a determinism cross
check, never the only oracle for what the values should be.
A test whose sole oracle is another run passes for as long as the bug is consistent, so it
reports green while the behaviour drifts and pins nothing at all.

Example: `crates/cerulion_core/tests/bounded_view_test.rs` builds every frame by hand,
byte by byte, from constants chosen in the test, and asserts each served value against
the literal that was written.

## 2. Prove the claim at the boundary the user feels

A zero copy claim is proven by ADDRESS, that the view or the written slot lies inside the
publisher's shared memory mapping, and ships beside a copy path control that fails the
same check; a latency claim is proven at the size and posture the user runs; a refusal is
proven by the message the user reads.
Byte equality, a passing parity check and a fast microbenchmark are all equally true of a
copy, so none of them proves the property the user came for.

Example: `crates/rmw_cerulion/tests/rmw_adopt_take_test.rs` pins the taken message to the
registered shared memory range by pointer identity, and its copy path control ends with
an empty registry.

## 3. Error paths run through a seam, not a mock

Reach an error arm through a seam that already exists in the production type, a function
parameter, a test hook or a fault counter, and drive it synchronously; never a sleep, and
a helper thread only where the error state cannot exist without one, joined before the
assertion.
A mock proves only that the mock behaves, and a sleep proves only that the clock moved,
which turns an ordering defect into an intermittent failure instead of a red test.

Example: `crates/cerulion_bag/tests/bag_late_channel_test.rs` fails a chunk write through
the writer's own fault counter, discards the chunk, and asserts the registration written
earlier as a top level record is still in the finished file while the message that chunk
held is not.

## 4. Architecture rules are unit tests

A rule such as "only one crate may depend on this tree" or "the default build stays clear
of that one" lives as a test over the workspace metadata, so a declared edge that is
optional, renamed or spelled as a subcrate fails `cargo test` rather than passing quietly.
A rule that lives only in a configuration file or a page of prose is enforced only where
someone thought to look, and a subcrate spelling walks straight past it.

Example: `crates/cerulion_cli_engine/tests/dependency_door_test.rs` derives each family of
crates at test time from the workspace metadata and fails any declared direct edge outside
the one door for that family; the resolved graph is the dependency policy's job.

## 5. Validate before any side effect, and assert the absence

Every rejection test ends by asserting that the file, directory, service or record was
NOT created, and every changed default arrives with an audit of the call sites that read
the old value plus a matrix test over the kinds the default reaches.
A refusal that already wrote half its output is a corruption defect wearing an error
message, and only the absence assertion tells the two apart.

Example: `crates/cerulion_cli_engine/tests/ros2_migrate_test.rs` checks the whole workspace
after a refusal, untracked files included, so a refusal that left a lock file or a stray
patch behind fails rather than passing on the error text alone.

## 6. Review threads close on evidence

Every reply names the commit that fixes the thread, the mechanism that was wrong and the
regression test that now pins it; a thread whose defect is still open stays open, and a
correct remark that is out of scope is recorded with the exact oracle to add.
Resolving a thread is a claim that the tree is fixed, and agreement costs nothing, so the
reply carries the proof instead.

Shape, no file yet: the thread reply and the commit it cites are the artifact here, and
the regression test named in the reply is the part that outlives the conversation.
