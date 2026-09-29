#!/usr/bin/env python3
"""Score a Phase 3 session: several entities, agents placed by the planner, the
device's see-through view. Works the same for a scored run and a played one.

Offline analysis only. Reads <logdir>/coop_bag and <logdir>/frames.

Overlay (per camera frame and entity, /device/overlay/truth):
  see-through coverage  of the entity-frames where the entity is in the
                        device's view but hidden from it, and some agent sees
                        it, the fraction with a track drawn on it
  pixel error, IoU, data age   as in Phase 2
Tracks (the device's picture of all entities at once, 10 Hz):
  At every track sample, the live tracks (confirmed, updated in the last
  0.5 s) are matched to entities (nearest first, within MATCH_RADIUS_M).
  Coasting tracks are predictions, which the overlay draws as such (grey,
  dashed); they are scored separately (prediction accuracy), not as false
  tracks. Over the moments an entity is visible to at least one agent:
  MOTA          1 - (misses + false tracks + ID switches) / entity-samples
                (the CLEAR MOT accuracy, restricted to what someone could see)
  MOTP          mean horizontal error of the matched tracks
  continuity    fraction of those entity-samples with a matched track
  ID switches   per entity, per minute
  prediction accuracy   share of coasting-track samples within MATCH_RADIUS_M
                of some entity
Planner:
  see-through availability  of the entity-time hidden from the device (in its
                        view), the fraction some agent could see: what the
                        planner's placement made possible at all
The overlay metrics and availability are scored within the planner's radius of
interest around the device (the "defined radius" the agents cover); the same
numbers over the device's whole field of view are reported alongside.

Outputs metrics.json, phase3.png and phase3_frames.png, prints the results,
and with --check exits non-zero if the device's view misses the thresholds.
"""
import argparse
import json
import sys
from bisect import bisect_right
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_phase1 as p1  # noqa: E402

MATCH_RADIUS_M = 2.0
TRACK_STALE_S = 0.25
CONFIRMED = (1, 2)
LIVE = (1,)

# Acceptance thresholds for --check, reasoning in docs/phase3_playable_scene.md.
THRESHOLDS = {
    "see_through_coverage": (">=", 0.85),
    "pixel_error_median_px": ("<=", 25.0),
    "age_p90_s": ("<=", 0.30),
    "mota": (">=", 0.70),
}


def visibility_by_entity(bag, agents):
    """{entity: (t, visible to >= 1 agent)} on the detectors' frame grid."""
    per = defaultdict(lambda: defaultdict(list))
    for n in agents:
        for m in bag.get(f"/agent_{n}/visibility_truth", []):
            per[m.entity][n].append((p1.stamp(m.stamp), m.visible))
    out = {}
    for ent, by_agent in per.items():
        grid = np.array(sorted(t for t, _ in by_agent[agents[0]]))
        any_v = np.zeros(len(grid), dtype=bool)
        for n, rows in by_agent.items():
            rows.sort()
            tv = np.array([t for t, _ in rows])
            vv = np.array([v for _, v in rows], dtype=float)
            any_v |= np.interp(grid, tv, vv) > 0.5
        out[ent] = (grid, any_v)
    return out


def match(tracks, truths, statuses=LIVE):
    """Greedy nearest matching of tracks to entities: {entity: track}."""
    pairs = []
    for ent, xy in truths.items():
        for tr in tracks:
            if tr[1] not in statuses:
                continue
            d = float(np.hypot(*(tr[3][:2] - xy[:2])))
            if d <= MATCH_RADIUS_M:
                pairs.append((d, ent, tr[0], tr))
    pairs.sort(key=lambda p: p[0])
    used_e, used_t, out = set(), set(), {}
    for d, ent, tid, tr in pairs:
        if ent in used_e or tid in used_t:
            continue
        used_e.add(ent)
        used_t.add(tid)
        out[ent] = (tr, d)
    return out


def mot(samples, gt, vis, t0, t1):
    ents = sorted(vis)
    vis_s = {e: p1.Series(vis[e][0], vis[e][1][:, None].astype(float)) for e in ents}
    n_gt = misses = fps = switches = 0
    errs = []
    last_id = {}
    matched_time = defaultdict(int)
    visible_time = defaultdict(int)
    coast_near = coast_all = 0
    for t, tracks in samples:
        if t < t0 or t > t1:
            continue
        truths = {e: gt[e].at(np.array([t]))[0] + np.array([0, 0, 0.9]) for e in ents if e in gt}
        m = match(tracks, truths)
        for e in ents:
            if vis_s[e].at(np.array([t]))[0, 0] <= 0.5:
                continue
            n_gt += 1
            visible_time[e] += 1
            if e in m:
                tr, d = m[e]
                errs.append(d)
                matched_time[e] += 1
                if e in last_id and last_id[e] != tr[0]:
                    switches += 1
                last_id[e] = tr[0]
            else:
                misses += 1
        matched_ids = {tr[0] for tr, _ in m.values()}
        fps += sum(1 for tr in tracks if tr[1] in LIVE and tr[0] not in matched_ids and
                   all(np.hypot(*(tr[3][:2] - xy[:2])) > MATCH_RADIUS_M for xy in truths.values()))
        for tr in tracks:
            if tr[1] == 2:
                coast_all += 1
                coast_near += any(np.hypot(*(tr[3][:2] - xy[:2])) <= MATCH_RADIUS_M for xy in truths.values())
    minutes = max((t1 - t0) / 60.0, 1e-9)
    return {
        "entity_samples": n_gt,
        "mota": 1.0 - (misses + fps + switches) / n_gt if n_gt else float("nan"),
        "motp_m": float(np.mean(errs)) if errs else float("nan"),
        "continuity": 1.0 - misses / n_gt if n_gt else float("nan"),
        "false_track_samples": fps,
        "id_switches": switches,
        "id_switches_per_entity_min": switches / len(ents) / minutes if ents else float("nan"),
        "prediction_accuracy": coast_near / coast_all if coast_all else float("nan"),
        "per_entity_continuity": {e: matched_time[e] / visible_time[e] if visible_time[e] else float("nan")
                                  for e in ents},
    }


def overlay(frames, vis, gt=None, radius=None):
    """Overlay metrics; with `radius`, only entity-frames within it of the device."""
    rows = []
    for m in frames:
        if m.entity not in vis:
            continue
        g, v = vis[m.entity]
        t = p1.stamp(m.stamp)
        if radius is not None:
            e = gt[m.entity].at(np.array([t]))[0]
            d = gt["device"].at(np.array([t]))[0]
            if np.hypot(e[0] - d[0], e[1] - d[1]) > radius:
                continue
        seen = np.interp(t, g, v.astype(float)) > 0.5
        rows.append((t, m.entity, m.in_frustum, m.hidden, seen, m.drawn, m.pixel_error, m.iou, m.age_s,
                     m.world_error_m, m.agents_seeing))
    if not rows:
        return {}, []
    r = np.array([(x[0], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10]) for x in rows], dtype=float)
    t, in_f, hid, seen, drawn, px, iou, age, werr, nag = r.T
    in_f, hid, seen, drawn = (x > 0.5 for x in (in_f, hid, seen, drawn))
    through = in_f & hid & seen
    live = through & drawn & (nag > 0)

    def med(mask, v, q=50):
        return float(np.percentile(v[mask], q)) if mask.any() else float("nan")

    hidden_in_view = in_f & hid
    return {
        "entity_frames_in_view": int(in_f.sum()),
        "entity_frames_see_through": int(through.sum()),
        "see_through_coverage": float(drawn[through].mean()) if through.any() else float("nan"),
        "see_through_availability": float(seen[hidden_in_view].mean()) if hidden_in_view.any() else float("nan"),
        "pixel_error_median_px": med(through & drawn, px),
        "pixel_error_p90_px": med(through & drawn, px, 90),
        "iou_median": med(through & drawn, iou),
        "world_error_median_m": med(through & drawn, werr),
        "age_median_s": med(live, age),
        "age_p90_s": med(live, age, 90),
    }, rows


def pick_frames(frames_dir, rows):
    files = sorted(frames_dir.glob("frame_*.png"))
    if not files or not rows:
        return []
    by_t = defaultdict(list)
    for x in rows:
        by_t[round(x[0], 2)].append(x)
    ts = np.array(sorted(by_t))
    cases = [("seen through walls", lambda xs: sum(1 for x in xs if x[2] and x[3] and x[5] and x[10] > 0)),
             ("several at once", lambda xs: sum(1 for x in xs if x[2] and x[5]))]
    out = []
    for title, count in cases:
        scored = []
        for f in files:
            tf = float(f.stem.split("_")[1])
            i = int(np.argmin(np.abs(ts - tf)))
            xs = by_t[ts[i]]
            scored.append((count(xs), f, tf))
        scored.sort(key=lambda s: -s[0])
        if scored and scored[0][0] > 0:
            out.append((f"{title} ({scored[0][0]} highlighted)", scored[0][1], scored[0][2]))
    return out


def plot(logdir, gt, bag, agents, occ_file, exclude, picks, entities):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.image as mpimg
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib unusable ({type(exc).__name__}), skipping plots", file=sys.stderr)
        return
    import xml.etree.ElementTree as ET
    fig, ax = plt.subplots(figsize=(9, 9))
    root = ET.parse(occ_file).getroot()
    for model in root.iter("model"):
        if model.findtext("static", "false").strip() != "true" or model.get("name") in exclude:
            continue
        pose = [float(v) for v in (model.findtext("pose") or "0 0 0 0 0 0").split()]
        for col in model.iter("collision"):
            box = col.find("geometry/box/size")
            if box is not None:
                sx, sy, _ = (float(v) for v in box.text.split())
                c, s = np.cos(pose[5]), np.sin(pose[5])
                pts = [(pose[0] + c * u - s * v, pose[1] + s * u + c * v)
                       for u, v in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]
                ax.add_patch(Polygon(pts, color="0.6"))
            sph = col.find("geometry/sphere/radius")
            if sph is not None:
                ax.add_patch(plt.Circle((pose[0], pose[1]), float(sph.text), color="tab:green", alpha=0.3))
    for e in entities:
        if e in gt:
            ax.plot(gt[e].v[:, 0], gt[e].v[:, 1], lw=1, alpha=0.7, label=e)
    if "device" in gt:
        ax.plot(gt["device"].v[:, 0], gt["device"].v[:, 1], "k-", lw=2, label="device")
    for n in agents:
        k = f"agent_{n}"
        if k in gt:
            ax.plot(gt[k].v[:, 0], gt[k].v[:, 1], "--", lw=1, label=f"agent {n}")
    ax.set_aspect("equal")
    ax.legend(fontsize=7, loc="upper right")
    ax.set_title("Session from above: map, entities, device and agents (truth)")
    fig.tight_layout()
    fig.savefig(logdir / "phase3.png", dpi=110)
    plt.close(fig)
    if picks:
        fig, axs = plt.subplots(1, len(picks), figsize=(8 * len(picks), 6.2))
        for a, (title, f, t) in zip(np.atleast_1d(axs), picks):
            a.imshow(mpimg.imread(f))
            a.set_title(f"{title}, t={t:.1f} s", fontsize=11)
            a.axis("off")
        fig.tight_layout()
        fig.savefig(logdir / "phase3_frames.png", dpi=100)
        plt.close(fig)
    print(f"plots: {logdir / 'phase3.png'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    import yaml

    sc = yaml.safe_load(open(a.logdir / "scenario.yaml"))
    agents = [ag["id"] for ag in sc["agents"]]
    entities = [e["name"] for e in sc["entities"]]
    bag = p1.read_bag(a.logdir / "coop_bag")
    gt = p1.ground_truth(bag.get("/sim/ground_truth", []))
    vis = visibility_by_entity(bag, agents)
    radius = float(sc["planner"]["interest_radius_m"])
    ov, rows = overlay(bag.get("/device/overlay/truth", []), vis, gt, radius)
    ov_all, rows_all = overlay(bag.get("/device/overlay/truth", []), vis)
    t_all = np.concatenate([g for g, _ in vis.values()])
    t0, t1 = float(t_all.min()), float(t_all.max())
    samples = {"device": p1.track_samples(bag.get("/device/tracks", []))}
    for n in agents:
        samples[f"agent_{n}"] = p1.track_samples(bag.get(f"/baseline_agent_{n}/tracks", []))
    tracks = {k: mot(v, gt, vis, t0, t1) for k, v in samples.items()}
    diags = p1.last_diagnostics(bag.get("/diagnostics", []))
    fus = diags.get("/device/track_fusion: fusion", {})
    ov["mota"] = tracks["device"]["mota"]

    print(f"session {t1 - t0:.0f} s, {len(entities)} entities, {len(agents)} agents")
    print(f"  device view (overlay, per entity and camera frame), within {radius:.0f} m of the device"
          f"   [whole field of view]")

    def line(label, key, fmt, unit=""):
        v = ov.get(key, float("nan"))
        lim = THRESHOLDS.get(key)
        flag = ""
        if lim is not None and a.check:
            okv = (v >= lim[1]) if lim[0] == ">=" else (v <= lim[1])
            flag = "  ok" if okv else f"  FAIL ({lim[0]} {lim[1]})"
        va = ov_all.get(key, float("nan"))
        print(f"    {label:<46}{format(v, fmt):>10}  {unit:<3}[{format(va, fmt)}]{flag}")

    line("see-through availability (planner)", "see_through_availability", ".3f")
    line("see-through coverage (hidden, agents see it)", "see_through_coverage", ".3f")
    line("pixel error, see-through, median", "pixel_error_median_px", ".1f", "px")
    line("pixel error, see-through, p90", "pixel_error_p90_px", ".1f", "px")
    line("box IoU, see-through, median", "iou_median", ".2f")
    line("world error, see-through, median", "world_error_median_m", ".2f", "m")
    line("data age, live, median", "age_median_s", ".2f", "s")
    line("data age, live, p90", "age_p90_s", ".2f", "s")
    names = ["device"] + [f"agent_{n}" for n in agents]
    print("  tracks (all entities)" + "".join(f"{n.replace('_', ' '):>14}" for n in names))
    for label, key, fmt in (("MOTA", "mota", ".3f"), ("MOTP [m]", "motp_m", ".3f"),
                            ("continuity", "continuity", ".3f"), ("ID switches", "id_switches", "d"),
                            ("false track samples", "false_track_samples", "d"),
                            ("prediction accuracy", "prediction_accuracy", ".3f")):
        flag = ""
        if key == "mota" and a.check:
            flag = "  ok" if tracks["device"][key] >= THRESHOLDS["mota"][1] else "  FAIL"
        print(f"    {label:<30}" + "".join(f"{format(tracks[n][key], fmt):>14}" for n in names) + flag)
    print("    per-entity continuity (device): " + ", ".join(
        f"{e} {v:.2f}" for e, v in tracks["device"]["per_entity_continuity"].items()))
    if fus:
        print(f"  device fusion: ingest callback mean {float(fus.get('detection_cb_mean_us', 'nan')):.1f} us, "
              f"max {fus.get('detection_cb_max_us')} us, overruns {fus.get('detection_cb_overruns')}, "
              f"hot-path allocations {fus.get('hot_path_allocations')}")

    json.dump({"overlay": ov, "overlay_whole_view": ov_all, "interest_radius_m": radius, "tracks": tracks, "diagnostics": diags,
               "thresholds": {k: list(v) for k, v in THRESHOLDS.items()}},
              open(a.logdir / "metrics.json", "w"), indent=2, default=float)
    plot(a.logdir, gt, bag, agents, sc["world_file"], [e["model"] for e in sc["entities"]],
         pick_frames(a.logdir / "frames", rows_all), entities)
    failed = []
    for k, (op, lim) in THRESHOLDS.items():
        v = ov.get(k, float("nan"))
        if v != v or not ((v >= lim) if op == ">=" else (v <= lim)):
            failed.append(k)
    if a.check and failed:
        print(f"FAILED thresholds: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
