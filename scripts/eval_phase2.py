#!/usr/bin/env python3
"""Score Phase 2: the ground device's see-through overlay, against ground truth.

Offline analysis only. Reads the rosbag scripts/demo_phase2.sh records
(<logdir>/coop_bag) and the frames the overlay node saved (<logdir>/frames).

Two levels:
  overlay   per camera frame (15 Hz), from /device/overlay/truth: was the
            entity highlighted where it really is, in the device's own image?
  tracks    the device's fused track picture (/device/tracks) against each
            agent alone (/baseline_agent_<n>/tracks), with the Phase 1
            metrics (scripts/eval_phase1.py).

Overlay metrics (definitions in docs/phase2_device_overlay.md):
  see-through coverage  of the frames where the entity is in the device's
                        field of view but hidden from it AND some agent sees
                        it, the fraction with a track drawn on it
  hidden coverage       the same, whether or not an agent sees it (coasting
                        counts)
  pixel error           drawn box centre vs true box centre in the image
  IoU                   of the drawn and the true box in the image
  data age              frame time minus the newest detection in the drawn track
  time to first overlay first sighting by any agent -> first frame drawn

Outputs metrics.json, phase2.png and phase2_frames.png in the log directory,
prints the results, and with --check exits non-zero if the device misses the
acceptance thresholds below.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_phase1 as p1  # noqa: E402  (bag reading, track metrics)

# Acceptance thresholds for --check, reasoning in docs/phase2_device_overlay.md.
THRESHOLDS = {
    "see_through_coverage": (">=", 0.90),
    "pixel_error_median_px": ("<=", 20.0),
    "age_p90_s": ("<=", 0.30),
    "time_to_first_overlay_s": ("<=", 1.0),
    "track_rmse_visible_m": ("<=", 0.75),
}


def overlay_metrics(frames, vis_t, vis_any):
    """frames: list of OverlayTruth; vis_t/vis_any: agents' ground-truth visibility."""
    t = np.array([p1.stamp(m.stamp) for m in frames])
    agents_see = np.interp(t, vis_t, vis_any.astype(float)) > 0.5
    in_f = np.array([m.in_frustum for m in frames])
    hidden = np.array([m.hidden for m in frames])
    drawn = np.array([m.drawn for m in frames])
    px = np.array([m.pixel_error for m in frames])
    iou = np.array([m.iou for m in frames])
    age = np.array([m.age_s for m in frames])
    werr = np.array([m.world_error_m for m in frames])
    n_ag = np.array([m.agents_seeing for m in frames])

    def frac(mask):
        return float(drawn[mask].mean()) if mask.any() else float("nan")

    def stats(mask, v):
        sel = mask & drawn
        if not sel.any():
            return {"median": float("nan"), "p90": float("nan"), "n": 0}
        return {"median": float(np.median(v[sel])), "p90": float(np.percentile(v[sel], 90)),
                "n": int(sel.sum())}

    through = in_f & hidden & agents_see          # the see-through case
    hidden_any = in_f & hidden                    # including coasting
    visible = in_f & ~hidden                      # the device sees it itself
    fresh = through & (n_ag > 0)                  # drawn from live agent data
    first_vis = vis_t[np.argmax(vis_any)] if vis_any.any() else None
    first_drawn = t[np.argmax(drawn & (t >= (first_vis or 0)))] if drawn.any() else None
    return {
        "frames": int(len(frames)),
        "frames_in_view": int(in_f.sum()),
        "frames_see_through": int(through.sum()),
        "frames_device_sees": int(visible.sum()),
        "see_through_coverage": frac(through),
        "hidden_coverage": frac(hidden_any),
        "device_view_coverage": frac(visible),
        "pixel_error_median_px": stats(through, px)["median"],
        "pixel_error_p90_px": stats(through, px)["p90"],
        "pixel_error_device_sees_median_px": stats(visible, px)["median"],
        "iou_median": stats(through, iou)["median"],
        "world_error_median_m": stats(through, werr)["median"],
        "age_median_s": stats(fresh, age)["median"],
        "age_p90_s": stats(fresh, age)["p90"],
        "coasting_age_p90_s": stats(hidden_any & ~agents_see, age)["p90"],
        "time_to_first_overlay_s": float(first_drawn - first_vis)
        if first_vis is not None and first_drawn is not None else float("nan"),
        "_series": {"t": t, "in_f": in_f, "hidden": hidden, "drawn": drawn, "px": px,
                    "agents_see": agents_see, "n_ag": n_ag},
    }


def pick_frames(frames_dir, series):
    """One still per case (see-through with live data, coasting, the device's own
    view): the frame with the median overlay error of that case, so the
    picture is typical rather than the best one."""
    files = sorted(frames_dir.glob("frame_*.png"))
    if not files:
        return []
    ts = np.array([float(f.stem.split("_")[1]) for f in files])
    s = series

    def state_at(t):
        i = int(np.argmin(np.abs(s["t"] - t)))
        return i

    wanted = [("seen through the wall", lambda i: s["hidden"][i] and s["drawn"][i] and s["n_ag"][i] > 0),
              ("predicted, nobody sees it", lambda i: s["hidden"][i] and s["drawn"][i] and s["n_ag"][i] == 0),
              ("in the device's own view", lambda i: s["in_f"][i] and not s["hidden"][i] and s["drawn"][i])]
    out = []
    for title, ok in wanted:
        best = None
        for f, t in zip(files, ts):
            i = state_at(t)
            if ok(i):
                best = best or []
                best.append((s["px"][i], f, t))
        if best:
            best.sort(key=lambda b: b[0])
            px, f, t = best[len(best) // 2]
            out.append((title, f, t, px))
    return out


def plot(logdir, ov, picks):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.image as mpimg
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib unusable ({type(exc).__name__}), skipping plots", file=sys.stderr)
        return
    s = ov["_series"]
    t0 = s["t"][0]
    t = s["t"] - t0
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [1.2, 2, 1]})
    ax = axes[0]
    ax.fill_between(t, 0, s["in_f"] & s["hidden"], step="post", color="tab:cyan", alpha=0.3,
                    label="entity behind the wall (device's view)")
    ax.fill_between(t, 0, s["in_f"] & ~s["hidden"], step="post", color="tab:green", alpha=0.3,
                    label="device sees it itself")
    ax.fill_between(t, 1.05, 1.05 + 0.5 * s["agents_see"], step="post", color="tab:orange", alpha=0.5,
                    label="some agent sees it (truth)")
    ax.fill_between(t, 1.6, 1.6 + 0.5 * s["drawn"], step="post", color="k", alpha=0.6,
                    label="overlay drawn on it")
    ax.set_yticks([])
    ax.legend(fontsize=7, ncol=2, loc="upper left")
    ax.set_title("Device see-through overlay, per camera frame")
    ax = axes[1]
    px = np.where(s["drawn"], s["px"], np.nan)
    ax.plot(t, px, lw=0.8, color="tab:blue")
    ax.set_ylabel("overlay error [px]")
    ax.set_ylim(0, max(60, np.nanpercentile(px, 99) if np.isfinite(px).any() else 60))
    ax = axes[2]
    ax.step(t, s["n_ag"] * s["drawn"], where="post", color="k")
    ax.set_ylabel("agents feeding\nthe overlay")
    ax.set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(logdir / "phase2.png", dpi=110)
    plt.close(fig)
    if picks:
        fig, axs = plt.subplots(1, len(picks), figsize=(6.4 * len(picks), 5.2))
        axs = np.atleast_1d(axs)
        for a, (title, f, tt, px_err) in zip(axs, picks):
            a.imshow(mpimg.imread(f))
            a.set_title(f"{title}  (t={tt:.1f} s, overlay error {px_err:.0f} px)", fontsize=10)
            a.axis("off")
        fig.tight_layout()
        fig.savefig(logdir / "phase2_frames.png", dpi=100)
        plt.close(fig)
    print(f"plots: {logdir / 'phase2.png'}, {logdir / 'phase2_frames.png'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    import yaml

    bag = p1.read_bag(a.logdir / "coop_bag")
    gt = p1.ground_truth(bag.get("/sim/ground_truth", []))
    agents = sorted(int(p.stem.split("_")[1]) for p in (a.logdir / "params").glob("detector_*.yaml"))
    det = {n: yaml.safe_load(open(a.logdir / "params" / f"detector_{n}.yaml"))[f"/agent_{n}/detector"][
        "ros__parameters"] for n in agents}
    ref_h = det[agents[0]]["entity_ref_height_m"]

    # Agents' ground-truth visibility timeline (as in Phase 1).
    per = {n: sorted((p1.stamp(m.stamp), m.visible) for m in bag.get(f"/agent_{n}/visibility_truth", []))
           for n in agents}
    grid = np.array([t for t, _ in per[agents[0]]])
    counts = np.zeros(len(grid), dtype=int)
    for n in agents:
        tv = np.array([t for t, _ in per[n]])
        vv = np.array([v for _, v in per[n]], dtype=float)
        counts += (np.interp(grid, tv, vv) > 0.5).astype(int)
    vis_any = counts > 0

    ov = overlay_metrics(bag.get("/device/overlay/truth", []), grid, vis_any)

    first_vis = grid[np.argmax(vis_any)] if vis_any.any() else grid[0]
    t0, t1 = first_vis - 1.0, grid[-1]
    samples = {"device": p1.track_samples(bag.get("/device/tracks", []))}
    for n in agents:
        samples[f"agent_{n}"] = p1.track_samples(bag.get(f"/baseline_agent_{n}/tracks", []))
    tracks = {k: p1.score(v, gt["entity"], ref_h, grid, vis_any, t0, t1) for k, v in samples.items()}
    ov["track_rmse_visible_m"] = tracks["device"]["rmse_visible_m"]

    # ---- report
    print(f"device camera: {ov['frames']} frames; entity in view {ov['frames_in_view']}, of which "
          f"behind the wall while agents see it {ov['frames_see_through']}, visible to the device itself "
          f"{ov['frames_device_sees']}")

    def line(label, key, fmt, unit=""):
        v = ov[key]
        lim = THRESHOLDS.get(key)
        flag = ""
        if lim is not None and a.check:
            okv = (v >= lim[1]) if lim[0] == ">=" else (v <= lim[1])
            flag = "  ok" if okv else f"  FAIL ({lim[0]} {lim[1]})"
        print(f"  {label:<46}{format(v, fmt):>10}  {unit}{flag}")

    print("  overlay (device camera, ground truth from the simulator)")
    line("see-through coverage (hidden, agents see it)", "see_through_coverage", ".3f")
    line("hidden coverage (incl. coasting)", "hidden_coverage", ".3f")
    line("coverage while the device sees it itself", "device_view_coverage", ".3f")
    line("pixel error, see-through, median", "pixel_error_median_px", ".1f", "px")
    line("pixel error, see-through, p90", "pixel_error_p90_px", ".1f", "px")
    line("pixel error, device sees it, median", "pixel_error_device_sees_median_px", ".1f", "px")
    line("box IoU, see-through, median", "iou_median", ".2f")
    line("world error, see-through, median", "world_error_median_m", ".2f", "m")
    line("data age, live, median", "age_median_s", ".2f", "s")
    line("data age, live, p90", "age_p90_s", ".2f", "s")
    line("data age while coasting, p90", "coasting_age_p90_s", ".1f", "s")
    line("time to first overlay", "time_to_first_overlay_s", ".2f", "s")
    names = ["device"] + [f"agent_{n}" for n in agents]
    print("  tracks" + "".join(f"{n.replace('_', ' '):>14}" for n in names))
    for label, key, fmt in (("  RMSE while some agent sees it [m]", "rmse_visible_m", ".3f"),
                            ("  continuity", "continuity", ".3f"), ("  ID switches", "id_switches", "d"),
                            ("  time to first track [s]", "time_to_first_track_s", ".2f")):
        flag = ""
        if key == "rmse_visible_m" and a.check:
            flag = "  ok" if tracks["device"][key] <= THRESHOLDS["track_rmse_visible_m"][1] else "  FAIL"
        print(f"  {label:<32}" + "".join(f"{format(tracks[n][key], fmt):>14}" for n in names) + flag)

    metrics = {"overlay": {k: v for k, v in ov.items() if not k.startswith("_")},
               "tracks": {n: {k: v for k, v in r.items() if k != "series"} for n, r in tracks.items()},
               "diagnostics": p1.last_diagnostics(bag.get("/diagnostics", [])),
               "thresholds": {k: list(v) for k, v in THRESHOLDS.items()}}
    with open(a.logdir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2, default=float)
    plot(a.logdir, ov, pick_frames(a.logdir / "frames", ov["_series"]))

    failed = []
    for key, (op, lim) in THRESHOLDS.items():
        v = ov[key]
        if v != v or not ((v >= lim) if op == ">=" else (v <= lim)):
            failed.append(key)
    if a.check and failed:
        print(f"FAILED thresholds: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
