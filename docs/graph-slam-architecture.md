# Graph SLAM architecture

The Graph SLAM package is being split into a thin ROS shell and a deterministic
mapping engine. The dependency direction is one way:

```text
ROS component / offline adapters
              |
              v
      GraphSlamApplication
              |
              v
     deterministic backend
              ^
              |
 clock, storage, and output ports
```

The backend must not include ROS APIs, read a clock, or access the filesystem.
Online and offline execution must enter through the same application API. ROS
messages, parameters, services, and executors belong at the outer boundary.

## Configuration and composition root

`GraphSlamConfig` is a source-private snapshot of the ROS parameters. Startup
has four explicit phases:

1. declare and load every parameter;
2. normalize compatibility values;
3. validate the complete snapshot;
4. compose the backend's domain-specific configuration objects.

The validated snapshot is immutable after construction. Adaptive edge weights
and other live values are separate runtime state. Pure builders in
`graph_slam_composition` translate the snapshot into descriptor, loop-search,
pose-graph, filtering, grid, and GNSS configuration. This keeps ROS parameter
names and defaults out of backend code and makes configuration mapping directly
unit-testable.

Filesystem setup, publishers, subscriptions, and services still occur in the
component during this first milestone. They are effects to move behind ports;
they are not configuration work.

## Migration milestones

### 1. Configuration and composition root

- one typed parameter snapshot and one startup validation path;
- pure domain configuration builders;
- immutable startup intent separated from runtime state;
- no hand-written backend DTO copying in the ROS component.

### 2. Unified application

`GraphSlamApplication` is now the ordered, ROS-free mapping workflow entry
point. Both the live component and deterministic bag replay submit the same
ordered submap prefixes through `processSubmaps()`. The application owns the
query cursor, stride scheduling, accepted/deduplicated loop-edge set, and the
pose-graph optimization command. This makes callback batching observationally
equivalent to submitting one submap at a time.

The adapters still convert ROS messages, load clouds, emit logs, and publish or
write results. Registration, voxel filtering, descriptor storage, and 3D-BBS
objects are injected into the application during this milestone so their
ownership can move as one coherent unit in milestone 3.

### 3. Backend engine ownership

- make the application own graph, registration, optimization, and scheduling;
- expose immutable result snapshots instead of shared mutable containers;
- keep ordering and tie-breaking explicit for deterministic output.

### 4. External I/O ports

- define clock, cache/storage, diagnostics, and map-output ports;
- provide ROS/filesystem adapters outside the backend;
- use in-memory adapters for deterministic unit and replay tests.

### 5. Architecture hardening

- keep the ROS component at 300 lines or fewer;
- enforce forbidden dependencies for backend targets in CI;
- require online/offline parity and byte-identical loop-edge and trajectory
  artifacts for identical ordered input;
- run default CI and release gates for every milestone.

Each milestone is delivered as one reviewable pull request. Compatibility is
preserved at its boundary: existing parameter names, defaults, topics, and
services remain stable unless a migration is explicitly documented.
