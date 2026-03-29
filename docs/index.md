# lidarslam_ros2 Docs

<section class="hero">
  <div class="hero__copy">
    <div class="hero__eyebrow">ROS 2 LiDAR SLAM Docs</div>
    <h1>Pointcloud-map authoring, benchmark evidence, and browser-first map proof.</h1>
    <p>
      <code>lidarslam_ros2</code> is organized around a practical public path:
      build a pointcloud map, validate it, and open it through
      Autoware-compatible map workflows.
    </p>
    <div class="hero__badges">
      <span>RKO-LIO frontend</span>
      <span>graph_based_slam backend</span>
      <span>Foxglove proof path</span>
    </div>
    <div class="hero__actions">
      <a class="md-button md-button--primary" href="autoware-map-authoring.html">Start With Map Authoring</a>
      <a class="md-button" href="autoware-quickstart.html">Run The Quickstart</a>
    </div>
  </div>
  <div class="hero__visual">
    <img src="assets/images/autoware_map_loader_proof.png" alt="Browser proof of an Autoware-compatible pointcloud map" />
  </div>
</section>

<section class="proof-grid">
  <article class="proof-card">
    <h2>Autoware-compatible proof</h2>
    <p>
      The public flow publishes a live <code>/map/pointcloud_map</code>,
      writes <code>map_projector_info.yaml</code>, and keeps map verification
      in the documented path.
    </p>
    <a href="autoware-foxglove.html">Open the Foxglove viewer path</a>
  </article>
  <article class="proof-card">
    <h2>Map cleanup with evidence</h2>
    <p>
      Save-time dynamic filtering reduces map size while preserving coarse
      footprint overlap. The validation reports track both reduction and tile
      overlap.
    </p>
    <img src="assets/images/dynamic_object_filter_bag6_summary.svg" alt="Dynamic-object filter benchmark summary" />
  </article>
</section>

## Start Here

<div class="card-grid">
  <a class="link-card" href="autoware-map-authoring.html">
    <h3>Autoware-Compatible Map Authoring</h3>
    <p>The shortest product-level summary of the supported public path.</p>
  </a>
  <a class="link-card" href="autoware-quickstart.html">
    <h3>Autoware Quickstart</h3>
    <p>Go from bag preflight to verified pointcloud-map output.</p>
  </a>
  <a class="link-card" href="autoware-foxglove.html">
    <h3>Autoware Foxglove</h3>
    <p>Open the map loader output in a browser-first viewer path.</p>
  </a>
</div>

## Operations

<div class="card-grid">
  <a class="link-card" href="workflows.html">
    <h3>Operator Workflows</h3>
    <p>Required topics, optional GNSS, packet paths, and map-save flows.</p>
  </a>
  <a class="link-card" href="benchmarking.html">
    <h3>Benchmarking And Release Gate</h3>
    <p>Run the tracked benchmark suite and generate the published reports.</p>
  </a>
  <a class="link-card" href="comparison.html">
    <h3>Comparison</h3>
    <p>See the current public position and benchmark-backed configuration summary.</p>
  </a>
</div>

## Current Snapshot

| Area | Current public position |
| --- | --- |
| Main path | `RKO-LIO` + `graph_based_slam` |
| Public map output | `pointcloud_map/` + `map_projector_info.yaml` |
| Browser proof | Foxglove path documented and smoke-tested |
| Long-loop evidence | `MID360` |
| Ground-truth benchmark | `NTU VIRAL tnp_01` |
| Save-time cleanup | dynamic filter with cross-dataset validation |

## Releases

- [v0.2.2](releases/v0.2.2.md)
- [v0.2.1](releases/v0.2.1.md)
- [v0.2.0](releases/v0.2.0.md)

## Local Preview

Build the docs:

```bash
python3 -m mkdocs build --strict
```

Serve them locally:

```bash
python3 -m mkdocs serve
```
