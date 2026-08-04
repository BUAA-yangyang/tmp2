#!/usr/bin/env python3
"""Render what opening_bearing actually saw, per attempt.

The question is why floor 2 chose a 92 degree correction while floor 1 chose
11. That is answerable only from the grid and the ray profile the function
was given, so this prints both rather than reasoning about the code.
"""
import json
import math
import sys


def render(record):
    crop = record["crop"]
    half = record["crop_half_cells"]
    res = record["crop_resolution"]
    yaw = record["robot_yaw"]
    bearing = record.get("bearing")
    # crop rows go +y upward; print top-down so north is up
    glyph = {-2: " ", -1: "?"}
    lines = []
    for r in range(len(crop) - 1, -1, -1):
        row = crop[r]
        if row is None:
            lines.append("~" * (2 * half + 1))
            continue
        out = []
        for c, v in enumerate(row):
            dx = (c - half) * res
            dy = (r - half) * res
            if abs(dx) < res and abs(dy) < res:
                out.append("R")
                continue
            ch = glyph.get(v)
            if ch is None:
                ch = "#" if v >= 65 else ("." if v <= 20 else "+")
            # mark the chosen bearing and the robot heading
            ang = math.atan2(dy, dx)
            dist = math.hypot(dx, dy)
            if bearing is not None and dist > 0.3:
                if abs(math.atan2(math.sin(ang - bearing),
                                  math.cos(ang - bearing))) < 0.04:
                    ch = "B"
            if dist > 0.3 and abs(math.atan2(math.sin(ang - yaw),
                                             math.cos(ang - yaw))) < 0.04:
                ch = "H" if ch != "B" else "*"
            out.append(ch)
        lines.append("".join(out))
    return lines


def histogram(rays, bins=24):
    """Free run vs bearing, folded into coarse sectors."""
    buckets = [[] for _ in range(bins)]
    for b, r in rays:
        idx = int((b + math.pi) / (2 * math.pi) * bins) % bins
        buckets[idx].append(r)
    out = []
    for i, vals in enumerate(buckets):
        centre = -180 + (360.0 / bins) * (i + 0.5)
        m = max(vals) if vals else 0.0
        out.append((centre, m))
    return out


def main(path):
    records = [json.loads(l) for l in open(path) if l.strip()]
    print("%d attempt(s) in %s\n" % (len(records), path))
    for rec in records:
        print("=" * 74)
        print("floor=%d attempt=%d  sim_t=%.2f  robot=(%.2f, %.2f) yaw=%.1f deg"
              % (rec["floor"], rec["attempt"], rec["sim_time"],
                 rec["robot_xy"][0], rec["robot_xy"][1],
                 math.degrees(rec["robot_yaw"])))
        print("  map known cells: %d / %d  (%.4f%% of grid)"
              % (rec["grid_known_cells"], rec["grid_total_cells"],
                 100.0 * rec["grid_known_fraction"]))
        if "median_run" in rec:
            print("  rays: median=%.3f m  best=%.3f m  contrast=%.3f m "
                  "(needs >= %.2f)  -> %s"
                  % (rec["median_run"], rec["best_run"], rec["contrast"],
                     rec["min_contrast"],
                     "ACCEPTED" if rec.get("accepted") else "rejected"))
            print("  rays at max_range %.1f m: %d/180 (%.1f%%)"
                  % (rec["max_range"], rec["at_max_range"],
                     100.0 * rec["capped_fraction"]))
        if rec.get("accepted"):
            print("  averaged lobe: %d rays, concentration=%.3f "
                  "(1.0 = all same direction)  -> bearing %.1f deg"
                  % (rec["good_count"], rec["lobe_concentration"],
                     math.degrees(rec["bearing"])))
            gb = [math.degrees(b) for b in rec["good_bearings"]]
            print("  averaged bearings span: %.1f .. %.1f deg"
                  % (min(gb), max(gb)))
            # detect split lobes
            gaps = [(gb[i + 1] - gb[i], gb[i], gb[i + 1])
                    for i in range(len(gb) - 1)]
            big = [g for g in gaps if g[0] > 20.0]
            if big:
                print("  !! SPLIT LOBES: %d gap(s) > 20 deg -> the circular "
                      "mean lands BETWEEN them" % len(big))
                for g, a, b in big:
                    print("       gap %.1f deg between %.1f and %.1f" % (g, a, b))
        print("\n  free run by sector (m):")
        for centre, m in histogram(rec["rays"]):
            bar = "#" * int(m / rec["max_range"] * 40)
            print("   %7.1f deg %5.2f %s" % (centre, m, bar))
        print("\n  grid crop (R=robot, #=occupied, .=free, ?=unknown, "
              "B=chosen bearing, H=heading, *=both), 1 char = %.3f m:"
              % rec["crop_resolution"])
        for line in render(rec):
            print("   " + line)
        print()


if __name__ == "__main__":
    main(sys.argv[1])
