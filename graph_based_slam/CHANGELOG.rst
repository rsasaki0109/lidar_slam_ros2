^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package graph_based_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-12)
------------------
* Deterministic core / ROS shell refactor (v0.6 roadmap): the loop-closure
  pipeline (descriptor ingestion, candidate generation/rerank, verification,
  accepted-edge set, pose-graph optimization, map saving) now lives in
  tested pure headers assembled by a ROS-free ``BackendCore``.
* Event-driven loop search is the default (searches on submap arrival);
  the ``deterministic_loop_scheduling`` parameter is retired and the legacy
  wall-clock timer path stays behind ``event_driven_loop_search: false``
  for one release.
* New ``graph_slam_offline_runner``: replays a recorded odometry bag through
  the same core with no executor and no wall clock; three runs produce
  byte-identical loop edges and optimized trajectories on both release-gate
  substrates.
* Input path hardened: odometry+cloud subscriptions synchronized
  (``message_filters``) with a deep queue — fixes a silent ~6% frame drop —
  and IMU/GNSS graph edges now weight the correct EdgeSE3 error blocks.
* GNSS yaw alignment with gauge release: maps are georeferenced in the
  projector's local ENU frame when GNSS is enabled.
* Contributors: Ryohei Sasaki

0.5.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* SPDX license tag (BSD-2-Clause).
* ``/map_save`` writes the Autoware map bundle: tiled ``pointcloud_map/`` with
  metadata plus ``map_projector_info.yaml`` (``local``, or ``LocalCartesian``
  when GNSS is enabled).
* Loop closure: NIS-driven adjacent-edge auto scaling, fitness-weighted loop
  edges, opt-in deterministic loop scheduling
  (``deterministic_loop_scheduling``, default off), and an opt-in BSD-2
  triangle-descriptor candidate source (``use_triangle_descriptor``, default
  off).
* Save-time dynamic-object cleanup and optional GNSS constraints.
* Contributors: Ryohei Sasaki
