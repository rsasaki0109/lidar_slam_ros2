#!/usr/bin/env bash
# md5 of a pipeline run's outputs: trajectory, pose graph, loop edges, map.pcd, and one hash over every
# pointcloud_map tile (file name and content). Usage: out_hashes.sh <run_dir>...
for d in "$@"; do
  t=$(md5sum < "${d}/trajectory_optimized.tum" | cut -c1-32)
  g=$(md5sum < "${d}/pose_graph.g2o" | cut -c1-32)
  l=$(md5sum < "${d}/loop_edges.csv" | cut -c1-32)
  m=$(md5sum < "${d}/map.pcd" | cut -c1-32)
  tiles=$( (cd "${d}/pointcloud_map" && md5sum -- * | sort -k2 | md5sum | cut -c1-32) )
  n=$(ls "${d}/pointcloud_map" | wc -l)
  echo "$(basename "${d}") traj=${t} g2o=${g} loops=${l} map=${m} tiles=${tiles} (${n} files)"
done
