# SOTA v7 RKO-LIO configurations

These files hold RKO-LIO algorithm keys only. Combine each with a rig's own keys (topics, frames, extrinsics, IMU
acceleration unit), as the headers say.

- `rko_lio_global_G12_2026-10-26.yaml`: the Phase 1 global configuration G12 (`bench_report/REPORT.md` chapter 31.3).
- `rko_lio_grandtour_hs_G15g_2026-10-28.yaml`: the GrandTour Hesai XT-32 + STIM320 configuration HS-G15g (chapter 35).
  It is for that rig only, not a global configuration.
- The other files are earlier revisions (G, G15 to G15f), kept as the record.

Do not edit these files, not even their comments. The COMFORT preregistrations and decision rules under
`bench_report/data/grandtour/` name several of them by md5.

`Thirdparty/rko_lio` carries every key these files set from `7e91814` on ([rsasaki0109/rko_lio#43](https://github.com/rsasaki0109/rko_lio/pull/43)).
The rko_lio commits named in the file headers are in that history. That merge also removed the experiments these
files do not use (report chapter 40). An older rko_lio build ignores the keys it does not know, without an error.
