"""Summarize PROF lines (temporary per-alignment instrumentation of the live NDT loop verification)."""
import re
import statistics
import sys

for path in sys.argv[1:]:
    rows = []
    for line in open(path):
        m = dict(re.findall(r'(\w+)=([-0-9.]+)', line))
        if 'iters' in m:
            rows.append({k: float(v) for k, v in m.items()})
    if not rows:
        continue
    it = [r['iters'] for r in rows]
    al = [r['align_ms'] for r in rows]
    total = sum(r['set_ms'] + r['align_ms'] + r['fit_ms'] for r in rows)
    capped = sum(1 for v in it if v >= 100)
    print(f"{path}: alignments {len(rows)}, source pts median {statistics.median(r['src'] for r in rows):.0f}, "
          f"target pts median {statistics.median(r['tgt'] for r in rows):.0f}")
    print(f"  iterations median {statistics.median(it):.0f}, mean {statistics.mean(it):.1f}, max {max(it):.0f}, "
          f"at the 100 cap {capped} ({100 * capped / len(rows):.0f}%)")
    print(f"  align wall ms median {statistics.median(al):.0f}, mean {statistics.mean(al):.0f}; "
          f"setInputTarget ms median {statistics.median(r['set_ms'] for r in rows):.1f}; "
          f"getFitnessScore ms median {statistics.median(r['fit_ms'] for r in rows):.1f}")
    print(f"  total wall in setInputTarget+align+fitness {total / 1000:.1f} s; fitness < 1.0: "
          f"{sum(1 for r in rows if r['fitness'] < 1.0)}")
