^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package graph_based_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
