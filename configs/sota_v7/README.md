# SOTA v7 RKO-LIO configurations

These files hold RKO-LIO algorithm keys only. Combine each with a rig's own keys (topics, frames, extrinsics, IMU
acceleration unit), as the headers say.

- `rko_lio_global_G22_2026-11-10.yaml`: the global configuration G since 2026-11-10 (chapter 61,
  `bench_report/data/memory/compact_2026-11-10/decision_rule_M1.txt`): the G12 keys plus `local_map_compact_points`
  (the local map's points as 32-bit offsets from their voxel's corner, 22-37 % less peak memory). It needs
  `Thirdparty/rko_lio` a232df3 or later.
- `rko_lio_global_G12_2026-10-26.yaml`: the Phase 1 global configuration G12 (`bench_report/REPORT.md` chapter 31.3),
  G until 2026-11-10.
- `rig_tactical_gyro_2026-11-09.yaml`: a per-rig input of G since G21 (chapter 60.9,
  `bench_report/data/phase1_global/decision_rule_G21.txt`). Add it on top of the G file for a rig whose gyro's
  datasheet publishes an angle random walk of at most 0.2 deg/sqrt(h) and an in-run bias instability of at most
  1 deg/h (the STIM320 of GrandTour). Leave it off for every other rig, like the gravity window (chapter 30.9). It
  needs `Thirdparty/rko_lio` a3828fa or later.
- `rko_lio_grandtour_hs_G15h_2026-11-09.yaml`: the GrandTour Hesai XT-32 + STIM320 configuration HS-G15h (chapter
  60.6): HS-G15g plus a per-scan tilt-correction limit and start-window gyro bias tracking. It is for that rig only,
  not a global configuration; it needs `Thirdparty/rko_lio` a3828fa or later.
- The other files are earlier revisions (G, G15 to G15g), kept as the record.

Do not edit these files, not even their comments. The COMFORT preregistrations and decision rules under
`bench_report/data/grandtour/` name several of them by md5.

`Thirdparty/rko_lio` carries every key these files set from `7e91814` on ([rsasaki0109/rko_lio#43](https://github.com/rsasaki0109/rko_lio/pull/43)).
The rko_lio commits named in the file headers are in that history. That merge also removed the experiments these
files do not use (report chapter 40). An older rko_lio build ignores the keys it does not know, without an error.
