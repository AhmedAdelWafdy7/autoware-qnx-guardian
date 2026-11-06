## What

<!-- One or two sentences: what this PR changes. -->

## Why

<!-- The problem it solves or the requirement it implements. -->

## How it was verified

- [ ] Host unit tests pass (`ctest --preset host-test`)
- [ ] QNX 8.0 cross-build passes (`cmake --build --preset qnx-aarch64le`), if `guardian/` changed
- [ ] On-target run (Pi 5 / QEMU), if timing or QNX APIs changed — attach output

## Checklist

- [ ] Commits are signed off (`git commit -s`)
- [ ] Safety limits changed? Each is a named constant with units and rationale
- [ ] README status / evidence table updated, if applicable
