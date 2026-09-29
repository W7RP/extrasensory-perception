#!/usr/bin/env python3
"""Tabulate and plot Phase 4's comparison from scored sessions.

  compare_phase4.py <compare_dir>            # reads <compare_dir>/sessions.txt ("label logdir" lines)
  compare_phase4.py <compare_dir> <label>=<logdir> ...

Reads each session's metrics.json (scripts/eval_phase4.py) and writes
comparison.md, comparison.json and comparison.png into <compare_dir>.
"""
import json
import sys
from pathlib import Path

NAN = float("nan")


def load(compare_dir, pairs):
    if not pairs:
        pairs = [ln.split(None, 1) for ln in (compare_dir / "sessions.txt").read_text().splitlines() if ln.strip()]
    rows = []
    for label, logdir in pairs:
        m = json.load(open(Path(logdir.strip()) / "metrics.json"))
        p4, ov, dev = m["phase4"], m["overlay"], m["tracks"]["device"]
        img = p4.get("imaging", {})
        wide, zoom = img.get("wide", {}), img.get("zoom", {})
        rows.append({
            "label": label, "logdir": str(logdir).strip(), "profile": p4["profile"], "agents": p4["agents"],
            "altitude_m": max(p4["altitude_m"]), "nav_bias_h_m": p4["nav_bias"]["horizontal_sigma_m"],
            "on_station_s": p4["window_s"][1] - p4["window_s"][0],
            "availability": ov.get("see_through_availability", NAN),
            "coverage": ov.get("see_through_coverage", NAN),
            "pixel_error_px": ov.get("pixel_error_median_px", NAN),
            "world_error_m": ov.get("world_error_median_m", NAN),
            "age_p90_s": ov.get("age_p90_s", NAN),
            "seen_near": p4["seen_near"], "seen_map": p4["seen_map"],
            "awareness_near": p4["awareness_near"], "awareness_map": p4["awareness_map"],
            "mota": dev["mota"], "motp_m": dev["motp_m"], "id_switches": dev["id_switches"],
            "det_err_median_m": p4["detection_error_median_m"], "det_err_p90_m": p4["detection_error_p90_m"],
            "wide_px": wide.get("pixels_median", NAN), "wide_p_detect": wide.get("p_detect_mean", NAN),
            "wide_identify": wide.get("identify", NAN), "zoom_share": zoom.get("share", NAN),
            "zoom_px": zoom.get("pixels_median", NAN),
            "zoom_on_target": p4.get("zoom", {}).get("on_target_share", NAN),
            "thermal": "thermal" in img,
        })
    return rows


COLUMNS = [
    ("session", "label", "{}"), ("agents", "agents", "{}"), ("alt [m]", "altitude_m", "{:.0f}"),
    ("nav err [m]", "nav_bias_h_m", "{:.2f}"), ("scored [s]", "on_station_s", "{:.0f}"),
    ("availability", "availability", "{:.3f}"), ("coverage", "coverage", "{:.3f}"),
    ("aware near", "awareness_near", "{:.3f}"), ("aware map", "awareness_map", "{:.3f}"),
    ("MOTA", "mota", "{:.3f}"), ("MOTP [m]", "motp_m", "{:.2f}"), ("px err", "pixel_error_px", "{:.1f}"),
    ("det err p50/p90 [m]", None, None), ("wide px", "wide_px", "{:.0f}"),
    ("wide P(det)", "wide_p_detect", "{:.2f}"), ("zoom px", "zoom_px", "{:.0f}"),
]


def fmt(v, f):
    if isinstance(v, float) and v != v:
        return "-"
    return f.format(v)


def table(rows):
    head = "| " + " | ".join(c[0] for c in COLUMNS) + " |"
    sep = "|" + "|".join("---" for _ in COLUMNS) + "|"
    lines = [head, sep]
    for r in rows:
        cells = []
        for name, key, f in COLUMNS:
            if key is None:
                cells.append(f"{fmt(r['det_err_median_m'], '{:.2f}')} / {fmt(r['det_err_p90_m'], '{:.2f}')}")
            else:
                cells.append(fmt(r[key], f))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def plot(rows, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib unusable ({type(exc).__name__}), skipping the plot", file=sys.stderr)
        return
    import numpy as np
    fig, axs = plt.subplots(1, 3, figsize=(17, 5))
    ax = axs[0]
    labels = [r["label"] for r in rows]
    x = np.arange(len(rows))
    for k, (key, name) in enumerate((("availability", "see-through availability"),
                                     ("awareness_near", "awareness near the device"),
                                     ("awareness_map", "awareness, whole map"))):
        ax.bar(x + (k - 1) * 0.27, [r[key] for r in rows], 0.27, label=name)
    ax.set_xticks(x, labels, rotation=30, ha="right")
    ax.set_ylim(0, 1.3)
    ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_title("What the device knows about")
    ax.legend(fontsize=8, loc="upper center", ncol=3)
    sweep = sorted((r for r in rows if r["profile"] == "high" and r["nav_bias_h_m"] == 0 and not r["thermal"]),
                   key=lambda r: r["altitude_m"])
    if sweep:
        alt = [r["altitude_m"] for r in sweep]
        ax = axs[1]
        ax.plot(alt, [r["wide_px"] for r in sweep], "o-", label="wide 4K, px on a person (median)")
        ax.plot(alt, [r["zoom_px"] for r in sweep], "s-", label="zoom, px on a person (median)")
        for lvl, name in ((2, "detect"), (8, "recognise"), (12.8, "identify")):
            ax.axhline(lvl, color="0.6", lw=0.8, ls=":")
            ax.text(alt[0], lvl * 1.06, name, fontsize=7, color="0.4")
        ax.set_yscale("log")
        ax.set_xlabel("altitude [m]")
        ax.set_title("Pixels on target (Johnson levels dotted)")
        ax2 = ax.twinx()
        ax2.plot(alt, [r["wide_p_detect"] for r in sweep], "g^--", label="wide: P(detect)")
        ax2.set_ylim(0, 1.05)
        ax2.set_ylabel("P(detect), wide camera", color="g")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper right")
        ax = axs[2]
        ax.plot(alt, [r["det_err_median_m"] for r in sweep], "o-", label="detection error, median")
        ax.plot(alt, [r["det_err_p90_m"] for r in sweep], "o--", label="detection error, p90")
        ax.plot(alt, [r["motp_m"] for r in sweep], "s-", label="fused track error (MOTP)")
        ax.plot(alt, [r["awareness_near"] for r in sweep], "k^-", label="awareness near the device")
        ax.set_xlabel("altitude [m]")
        ax.set_ylabel("error [m]; awareness [share]")
        ax.set_title("Altitude: accuracy and coverage")
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    out = Path(sys.argv[1])
    pairs = [a.split("=", 1) for a in sys.argv[2:]]
    rows = load(out, pairs)
    md = table(rows)
    (out / "comparison.md").write_text(md + "\n")
    json.dump(rows, open(out / "comparison.json", "w"), indent=2)
    plot(rows, out / "comparison.png")
    print(md)
    print(f"\n{out / 'comparison.md'}, comparison.json, comparison.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
