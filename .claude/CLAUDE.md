# Design Philosophy

## Composability

Build atomic tools and functions that are useful on their own, not shaped around the one caller you happen to have.
Define a component's interface in terms of its own domain (references in, references out; bytes in, bytes out),
never in terms of the context that calls it today.
A good smell test: it is usable and testable from a shell or a REPL with no knowledge of the larger system.
Push context-specific assembly (which files, which mappings, what environment) out to the edges (the caller, the build glue) and keep the core general.
Pass data explicitly as arguments or streams rather than reading ambient state or files by convention.
This is the Unix philosophy: small pieces doing one thing, joined by simple explicit interfaces instead of shared assumptions.
Prefer separation of concerns over convenience coupling, even when the coupling would save a few lines now.

## Declarative and Functional by default

Prefer declarative or functional code - side effects are hard to reason about - whereas clear intent, interfaces, and output are not.

# Style

For text: attempt one sentence per line, in docstrings, comments, and documentation.
Where sentences are too long, try to break on clauses (commas, usually) rather than a fixed width.

Trust `pre-commit` autoformatters to maintain line length, import order, and other basic style rules, do not spend any effort on it.

# Atomic diffs

After prototyping a feature, break it up into independent parts that can be reviewed and merged on their own.
Aim for small, clear PRs.
