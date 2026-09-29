#!/usr/bin/env python3
"""Score Phase 1 against ground truth: the fused tracker vs each agent alone.

Offline analysis only. Reads the rosbag scripts/demo_phase1.sh records
(<logdir>/coop_bag) and compares three trackers that are identical except for
their inputs: /fusion (both agents) and /baseline_agent_<n> (agent n alone).

Metrics, per tracker (definitions in docs/phase1_two_agent_fusion.md):
  position RMSE     horizontal error of the track matched to the entity, over
                    every published track sample (10 Hz), split into "while
                    some agent sees it" and "while nobody does" (coasting)
  continuity        fraction of the time the entity is visible to at least one
                    agent (ground truth) during which a track is matched to it
  ID switches       changes of the matched track's id
  time to first track   from the entity first becoming visible to any agent
                    to the first confirmed track on it
  NEES              mean normalised position error of the matched track (3 dof;
                    3 = the covariance is honest)
"Matched": the nearest confirmed or coasting track within MATCH_RADIUS_M of the
entity. Visibility is the detectors' own ground-truth flag (VisibilityTruth),
so all three trackers are scored against the same timeline.

Outputs metrics.json and phase1.png in the log directory, prints the results
table, and with --check exits non-zero if the fused tracker misses the
acceptance thresholds below (fixed before the first scored run).
"""
import argparse
import json
import sys
from bisect import bisect_right
from pathlib import Path

import numpy as np

MATCH_RADIUS_M = 2.0
TRACK_STALE_S = 0.25          # a tracks sample older than this does not count at time t
CONFIRMED = (1, 2)            # Track.STATUS_CONFIRMED, STATUS_COASTING

# Acceptance thresholds for --check (fused tracker only), with the reasoning in
# docs/phase1_two_agent_fusion.md ("Acceptance").
THRESHOLDS = {
    "continuity": (">=", 0.90),
    "rmse_visible_m": ("<=", 0.75),
    "id_switches": ("<=", 2),
    "time_to_first_track_s": ("<=", 1.0),
    "hot_path_allocations": ("<=", 0),
    "detection_cb_overruns": ("<=", 0),
}


# ---------------------------------------------------------------- bag reading

def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out.setdefault(topic, []).append(deserialize_message(data, types[topic]))
    return out


def stamp(t):
    return t.sec + 1e-9 * t.nanosec


class Series:
    """Time series of vectors with linear interpolation."""

    def __init__(self, t, v):
        order = np.argsort(t)
        self.t = np.asarray(t)[order]
        self.v = np.asarray(v)[order]

    def at(self, t):
        return np.stack([np.interp(t, self.t, self.v[:, k]) for k in range(self.v.shape[1])], -1)


def ground_truth(msgs):
    frames = {}
    for m in msgs:
        for tf in m.transforms:
            tr = tf.transform.translation
            frames.setdefault(tf.child_frame_id, ([], []))
            frames[tf.child_frame_id][0].append(stamp(tf.header.stamp))
            frames[tf.child_frame_id][1].append((tr.x, tr.y, tr.z))
    return {k: Series(*v) for k, v in frames.items()}


def track_samples(msgs):
    """[(t, [(id, status, mask, pos(3), cov(3x3))...])] sorted by t."""
    out = []
    for m in msgs:
        tracks = []
        for tr in m.tracks:
            c = tr.covariance
            cov = np.array([[c[0], c[1], c[2]], [c[1], c[3], c[4]], [c[2], c[4], c[5]]])
            tracks.append((tr.id, tr.status, tr.source_mask, np.array(tr.position), cov))
        out.append((stamp(m.stamp), tracks))
    out.sort(key=lambda s: s[0])
    return out


def last_diagnostics(msgs):
    """{status name: {key: value}} from the newest message of each status."""
    out = {}
    for m in msgs:
        for st in m.status:
            out[st.name] = {kv.key: kv.value for kv in st.values}
    return out


# ------------------------------------------------------------------- metrics

def match(tracks, entity_xyz):
    """Nearest confirmed/coasting track within MATCH_RADIUS_M (horizontally)."""
    best, best_d = None, MATCH_RADIUS_M
    for tr in tracks:
        if tr[1] not in CONFIRMED:
            continue
        d = float(np.hypot(*(tr[3][:2] - entity_xyz[:2])))
        if d <= best_d:
            best, best_d = tr, d
    return best


def score(samples, entity, ref_height, vis_t, vis_any, t0, t1):
    """Metrics for one tracker over the window [t0, t1]."""
    ts = [s[0] for s in samples]
    err_h, err_vis, err_occ, nees, ids, series = [], [], [], [], [], []
    confirmed_ids = set()
    vis_series = Series(vis_t, vis_any[:, None].astype(float))
    for t, tracks in samples:
        if t < t0 or t > t1:
            continue
        for tr in tracks:
            if tr[1] in CONFIRMED:
                confirmed_ids.add(tr[0])
        truth = entity.at(t) + np.array([0.0, 0.0, ref_height])
        m = match(tracks, truth)
        if m is None:
            series.append((t, np.nan, 0))
            continue
        e = m[3] - truth
        eh = float(np.hypot(e[0], e[1]))
        err_h.append(eh)
        visible_now = vis_series.at(np.array([t]))[0, 0] > 0.5
        (err_vis if visible_now else err_occ).append(eh)
        try:
            nees.append(float(e @ np.linalg.solve(m[4], e)))
        except np.linalg.LinAlgError:
            pass
        ids.append(m[0])
        series.append((t, eh, bin(m[2]).count("1")))
    # Continuity: at each ground-truth visibility frame with >= 1 agent seeing,
    # is there a fresh tracks sample with a matched track?
    have, need = 0, 0
    first_vis = None
    first_track = None
    for t, v in zip(vis_t, vis_any):
        if t < t0 or t > t1:
            continue
        if v and first_vis is None:
            first_vis = t
        i = bisect_right(ts, t) - 1
        matched = False
        if i >= 0 and t - ts[i] <= TRACK_STALE_S:
            truth = entity.at(ts[i]) + np.array([0.0, 0.0, ref_height])
            matched = match(samples[i][1], truth) is not None
        if matched and first_vis is not None and first_track is None:
            first_track = ts[i]
        if v:
            need += 1
            have += matched
    switches = sum(1 for a, b in zip(ids, ids[1:]) if a != b)
    rms = lambda x: float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")  # noqa: E731
    return {
        "rmse_m": rms(err_h),
        "rmse_visible_m": rms(err_vis),
        "rmse_occluded_m": rms(err_occ),
        "max_error_m": float(np.max(err_h)) if err_h else float("nan"),
        "continuity": have / need if need else float("nan"),
        "id_switches": switches,
        "track_ids": sorted(set(ids)),
        "false_tracks": len(confirmed_ids - set(ids)),
        "time_to_first_track_s": (first_track - first_vis) if first_track is not None and
        first_vis is not None else float("nan"),
        "nees_mean_3dof": float(np.mean(nees)) if nees else float("nan"),
        "matched_samples": len(err_h),
        "series": series,
    }


def detection_stats(msgs, entity, ref_height):
    err, nees = [], []
    for m in msgs:
        t = stamp(m.stamp)
        truth = entity.at(t) + np.array([0.0, 0.0, ref_height])
        for d in m.detections:
            e = np.array(d.position) - truth
            c = d.covariance
            cov = np.array([[c[0], c[1], c[2]], [c[1], c[3], c[4]], [c[2], c[4], c[5]]])
            err.append(float(np.hypot(e[0], e[1])))
            nees.append(float(e @ np.linalg.solve(cov, e)))
    return {
        "detections": len(err),
        "rmse_m": float(np.sqrt(np.mean(np.square(err)))) if err else float("nan"),
        "nees_mean_3dof": float(np.mean(nees)) if nees else float("nan"),
    }


def nav_stats(msgs, agent_gt, origin):
    if not msgs:
        return {"rmse_m": float("nan")}
    t = np.array([stamp(m.header.stamp) for m in msgs])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                  for m in msgs]) + np.asarray(origin)
    keep = (t >= agent_gt.t[0]) & (t <= agent_gt.t[-1])
    e = p[keep] - agent_gt.at(t[keep])
    eh = np.hypot(e[:, 0], e[:, 1])
    return {"rmse_m": float(np.sqrt(np.mean(eh ** 2))), "max_m": float(np.max(eh)),
            "alt_rmse_m": float(np.sqrt(np.mean(e[:, 2] ** 2)))}


# ------------------------------------------------------------------- report

def plot(path, gt, agents, samples_by_name, results, vis, building):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception as exc:  # noqa: BLE001  (NumPy 2 vs apt matplotlib, see environment.md)
        print(f"matplotlib unusable ({type(exc).__name__}), skipping plot", file=sys.stderr)
        return
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.5], height_ratios=[2, 2, 1])
    ax = fig.add_subplot(gs[:, 0])
    for (x0, x1, y0, y1) in building:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="0.6", zorder=1))
    e = gt["entity"]
    ax.plot(e.v[:, 0], e.v[:, 1], color="k", lw=3, alpha=0.3, label="entity (truth)")
    for n in agents:
        a = gt[f"agent_{n}"]
        ax.plot(a.v[:, 0], a.v[:, 1], lw=1, label=f"agent {n} (truth)")
    colors = {0: "0.5", 1: "tab:orange", 2: "tab:green"}
    pts = [(tr[3][0], tr[3][1], bin(tr[2]).count("1")) for _, trs in samples_by_name["fused"]
           for tr in trs if tr[1] in CONFIRMED]
    if pts:
        p = np.array(pts)
        for k, c in colors.items():
            sel = p[:, 2] == k if k < 2 else p[:, 2] >= 2
            ax.scatter(p[sel, 0], p[sel, 1], s=4, color=c, zorder=3,
                       label=f"fused track, {k}{'+' if k == 2 else ''} agent(s) seeing")
    ax.set_aspect("equal")
    ax.set_xlabel("x east [m]")
    ax.set_ylabel("y north [m]")
    ax.set_title("World, top down")
    ax.legend(fontsize=7, loc="lower left")

    vt, vn = vis
    t0 = vt[0]
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 1], sharex=ax2)
    ax4 = fig.add_subplot(gs[2, 1], sharex=ax2)
    for a_ in (ax2, ax3):
        for i in range(len(vt) - 1):
            if vn[i] > 0:
                a_.axvspan(vt[i] - t0, vt[i + 1] - t0, color=colors[min(vn[i], 2)], alpha=0.12, lw=0)
    for name, style in (("fused", "-"), ("agent_1", ":"), ("agent_2", "--")):
        s = results[name]["series"]
        if s:
            ts = np.array([x[0] for x in s]) - t0
            ax2.plot(ts, [x[1] for x in s], style, lw=1.2, label=name.replace("_", " "))
    ax2.set_ylabel("horizontal error [m]")
    ax2.set_title("Track error vs truth (background: agents seeing the entity, truth)")
    ax2.legend(fontsize=8)
    for name, y in (("fused", 2), ("agent_1", 1), ("agent_2", 0)):
        s = results[name]["series"]
        ts = np.array([x[0] for x in s]) - t0
        has = np.array([not np.isnan(x[1]) for x in s])
        ax3.scatter(ts[has], np.full(has.sum(), y), s=3, marker="|")
    ax3.set_yticks([0, 1, 2], ["agent 2 alone", "agent 1 alone", "fused"])
    ax3.set_title("Track on the entity exists")
    ax4.step(vt - t0, vn, where="post", color="k")
    ax4.set_yticks([0, 1, 2])
    ax4.set_ylabel("agents\nseeing")
    ax4.set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"plot: {path}")


def building_from_world(world_file, exclude):
    sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
    import scenario  # noqa: E402
    return [(b[0], b[1], b[2], b[3]) for b in scenario.building_boxes(world_file, exclude)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--check", action="store_true", help="exit non-zero if thresholds are missed")
    a = ap.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
    import scenario as scn  # noqa: E402
    import yaml  # noqa: E402

    bag = read_bag(a.logdir / "coop_bag")
    gt = ground_truth(bag.get("/sim/ground_truth", []))
    det_params = {}
    agents = sorted(int(p.stem.split("_")[1]) for p in (a.logdir / "params").glob("detector_*.yaml"))
    for n in agents:
        det_params[n] = yaml.safe_load(open(a.logdir / "params" / f"detector_{n}.yaml"))[
            f"/agent_{n}/detector"]["ros__parameters"]
    ref_h = det_params[agents[0]]["entity_ref_height_m"]

    # Ground-truth visibility timeline: agents seeing the entity, per frame.
    per_agent = {}
    for n in agents:
        per_agent[n] = sorted((stamp(m.stamp), m.visible) for m in bag.get(f"/agent_{n}/visibility_truth", []))
    grid = np.array([t for t, _ in per_agent[agents[0]]])
    counts = np.zeros(len(grid), dtype=int)
    for n in agents:
        tv = np.array([t for t, _ in per_agent[n]])
        vv = np.array([v for _, v in per_agent[n]], dtype=float)
        counts += (np.interp(grid, tv, vv) > 0.5).astype(int)
    vis_any = counts > 0

    # Window: from the first moment any agent sees the entity (minus a margin
    # for time-to-first-track) to the end of the recording.
    first_vis = grid[np.argmax(vis_any)] if vis_any.any() else grid[0]
    t0, t1 = first_vis - 1.0, grid[-1]
    sel = (grid >= t0) & (grid <= t1)

    samples = {"fused": track_samples(bag.get("/fusion/tracks", []))}
    for n in agents:
        samples[f"agent_{n}"] = track_samples(bag.get(f"/baseline_agent_{n}/tracks", []))
    results = {name: score(s, gt["entity"], ref_h, grid, vis_any, t0, t1) for name, s in samples.items()}

    diags = last_diagnostics(bag.get("/diagnostics", []))
    fused_diag = diags.get("/fusion/track_fusion: fusion", {})
    for key in ("hot_path_allocations", "detection_cb_overruns"):
        results["fused"][key] = int(fused_diag.get(key, -1))

    dets = {n: detection_stats(bag.get(f"/agent_{n}/detections", []), gt["entity"], ref_h) for n in agents}
    navs = {n: nav_stats(bag.get(f"/agent_{n}/eskf/odometry", []), gt[f"agent_{n}"],
                         det_params[n]["origin_world_enu"]) for n in agents}

    # ---- report
    share = [100.0 * np.mean(counts[sel] == k) for k in (0, 1)] + [100.0 * np.mean(counts[sel] >= 2)]
    print(f"window {t1 - t0:.1f} s; entity visible to 0 / 1 / 2 agents: "
          f"{share[0]:.0f} / {share[1]:.0f} / {share[2]:.0f} % of the time")
    names = ["fused"] + [f"agent_{n}" for n in agents]
    header = "".join(f"{('agent ' + n[6:] + ' only') if n != 'fused' else 'fused':>14}" for n in names)
    print(f"  {'metric':<34}{header}")

    def row(label, key, fmt, unit=""):
        vals = "".join(f"{format(results[n][key], fmt):>14}" for n in names)
        lim = THRESHOLDS.get(key)
        flag = ""
        if lim is not None and a.check:
            v = results["fused"][key]
            okv = (v >= lim[1]) if lim[0] == ">=" else (v <= lim[1])
            flag = "  ok" if okv else f"  FAIL (fused {lim[0]} {lim[1]})"
        print(f"  {label:<34}{vals}  {unit}{flag}")

    row("position RMSE, horizontal", "rmse_m", ".3f", "m")
    row("  while some agent sees it", "rmse_visible_m", ".3f", "m")
    row("  while nobody does (coasting)", "rmse_occluded_m", ".3f", "m")
    row("max horizontal error", "max_error_m", ".3f", "m")
    row("track continuity", "continuity", ".3f", "of visible time")
    row("ID switches", "id_switches", "d")
    row("false confirmed tracks", "false_tracks", "d")
    row("time to first track", "time_to_first_track_s", ".2f", "s")
    row("NEES mean (ideal 3)", "nees_mean_3dof", ".2f")
    for n in agents:
        d, nv = dets[n], navs[n]
        print(f"  agent {n}: {d['detections']} detections, error RMSE {d['rmse_m']:.3f} m, "
              f"NEES {d['nees_mean_3dof']:.2f}/3; ESKF navigation horizontal RMSE "
              f"{nv['rmse_m']:.3f} m (max {nv['max_m']:.3f}), altitude {nv['alt_rmse_m']:.3f} m")
    print("  fusion node, real time (ingest thread, from /diagnostics):")
    for name, d in sorted(diags.items()):
        if not name.endswith(": fusion"):
            continue
        print(f"    {name.split(':')[0]:<22} callbacks {d.get('detection_callbacks')}, mean "
              f"{float(d.get('detection_cb_mean_us', 'nan')):.1f} us, p99 {d.get('detection_cb_p99_us')} us, "
              f"max {d.get('detection_cb_max_us')} us, budget {d.get('detection_cb_budget_us')} us, "
              f"overruns {d.get('detection_cb_overruns')}, hot-path allocations "
              f"{d.get('hot_path_allocations')}, ingest-thread allocations {d.get('ingest_thread_allocations')}, "
              f"late batches {d.get('late_batches')}")
    for name, d in sorted(diags.items()):
        if name.endswith(": filter"):
            print(f"    {name.split(':')[0]:<22} IMU cb mean {float(d.get('imu_cb_mean_us', 'nan')):.1f} us, "
                  f"max {d.get('imu_cb_max_us')} us, overruns {d.get('imu_cb_overruns')}, hot-path "
                  f"allocations {d.get('hot_path_allocations')}")

    metrics = {
        "window_s": float(t1 - t0),
        "visible_share_pct": {"0": share[0], "1": share[1], "2+": share[2]},
        "trackers": {n: {k: v for k, v in r.items() if k != "series"} for n, r in results.items()},
        "detections": {str(n): v for n, v in dets.items()},
        "navigation": {str(n): v for n, v in navs.items()},
        "diagnostics": diags,
        "thresholds": {k: list(v) for k, v in THRESHOLDS.items()},
    }
    with open(a.logdir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2, default=float)

    world_file = det_params[agents[0]]["world_file"]
    plot(a.logdir / "phase1.png", gt, agents, samples, results, (grid, counts),
         building_from_world(world_file, det_params[agents[0]]["exclude_models"]))

    failed = []
    for key, (op, lim) in THRESHOLDS.items():
        v = results["fused"][key]
        if not ((v >= lim) if op == ">=" else (v <= lim)) or v != v:
            failed.append(key)
    fused_c = results["fused"]["continuity"]
    best_single = max(results[f"agent_{n}"]["continuity"] for n in agents)
    if not fused_c >= best_single:
        failed.append("continuity_vs_best_single_agent")
    if a.check and failed:
        print(f"FAILED thresholds: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
