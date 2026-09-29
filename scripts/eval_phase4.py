#!/usr/bin/env python3
"""Score a Phase 4 session: the same session metrics as Phase 3, plus what
the imaging model and the high agent's zoom camera did. Works for either
profile, so a high-unit session and a low-agents session on the same map
compare like for like (scripts/compare_phase4.py).

Offline analysis only. Reads <logdir>/coop_bag.

Scoring window: from the moment every agent is on station (within 10 % of
its altitude and 5 m of its planner goal; a climb to 100 m takes ~35 s
longer than one to 12 m) to the
planner's first `land` goal, so the profiles are compared doing their job,
not taking off or coming down (a minute, from 100 m).

Everything from Phase 3 (eval_phase3.py): see-through availability and
coverage, pixel error, data age, MOTA / MOTP / continuity, within the
planner's radius of interest around the device. Added:
  awareness     share of ALL entity-time, visible or not, with a live device
                track on the entity: within the radius of interest, and over
                the whole map (a high unit's wide view covers much more
                than the device needs, which is worth knowing)
  seen          share of entity-time some agent's camera could see (truth)
  detections    horizontal error of single detections against truth (the
                ground-intersection geolocation, including navigation error)
Imaging model only (per camera, over the entity-frames it saw best):
  pixels on target (median), detection probability (mean), and the Johnson
  level shares (detect / recognise / identify)
Zoom camera only:
  share of the window on a target, distinct tracks looked at, mean dwell and
  mean revisit interval per track

Outputs metrics.json (with a `phase4` section), phase4.png, prints the
results, and with --check exits non-zero if the Phase 3 thresholds fail.
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_phase1 as p1  # noqa: E402
import eval_phase3 as p3  # noqa: E402

JOHNSON = (2.0, 8.0, 12.8)   # detect, recognise, identify [px across the critical dimension]


def on_station(gt, sc, bag):
    """First time every agent is at its altitude (within 10 %) and at its
    planner goal (within 5 m): climbed, turned and flown to its spot."""
    t = []
    for a in sc["agents"]:
        s = gt.get(f"agent_{a['id']}")
        goals = [(p1.stamp(g.stamp), g.position[0], g.position[1])
                 for g in bag.get(f"/agent_{a['id']}/goal", []) if not g.land]
        if s is None or not goals:
            return None
        gt_ = np.array([g[0] for g in goals])
        found = None
        for k in range(len(s.t)):
            if s.v[k, 2] < 0.9 * float(a["altitude_m"]):
                continue
            i = int(np.searchsorted(gt_, s.t[k], side="right")) - 1
            if i >= 0 and np.hypot(s.v[k, 0] - goals[i][1], s.v[k, 1] - goals[i][2]) <= 5.0:
                found = float(s.t[k])
                break
        if found is None:
            return None
        t.append(found)
    return max(t)


def landing_time(bag, agents):
    """Stamp of the first `land` goal to any agent, if the session got that far."""
    t = [p1.stamp(g.stamp) for n in agents for g in bag.get(f"/agent_{n}/goal", []) if g.land]
    return min(t) if t else None


def awareness(samples, gt, entities, t0, t1, radius):
    """Share of entity-samples with a live track on them: near the device, everywhere."""
    near = near_hit = every = every_hit = 0
    for t, tracks in samples:
        if t < t0 or t > t1:
            continue
        truths = {e: gt[e].at(np.array([t]))[0] + np.array([0, 0, 0.9]) for e in entities if e in gt}
        m = p3.match(tracks, truths)
        d = gt["device"].at(np.array([t]))[0]
        for e, xyz in truths.items():
            hit = e in m
            every += 1
            every_hit += hit
            if np.hypot(xyz[0] - d[0], xyz[1] - d[1]) <= radius:
                near += 1
                near_hit += hit
    return {"awareness_near": near_hit / near if near else float("nan"),
            "awareness_map": every_hit / every if every else float("nan")}


def seen_share(bag, agents, gt, t0, t1, radius):
    vis = p3.visibility_by_entity(bag, agents)
    near = near_seen = every = every_seen = 0
    for e, (g, v) in vis.items():
        keep = (g >= t0) & (g <= t1)
        g, v = g[keep], v[keep]
        if not len(g) or e not in gt:
            continue
        xy = gt[e].at(g)[:, :2]
        dev = gt["device"].at(g)[:, :2]
        close = np.hypot(*(xy - dev).T) <= radius
        every += len(g)
        every_seen += int(v.sum())
        near += int(close.sum())
        near_seen += int(v[close].sum())
    return {"seen_near": near_seen / near if near else float("nan"),
            "seen_map": every_seen / every if every else float("nan")}


def detection_errors(bag, agents, gt, entities, t0, t1, ref_height):
    errs = []
    for n in agents:
        for m in bag.get(f"/agent_{n}/detections", []):
            t = p1.stamp(m.stamp)
            if t < t0 or t > t1 or not m.detections:
                continue
            truths = np.array([gt[e].at(np.array([t]))[0] for e in entities if e in gt])
            for d in m.detections:
                dd = np.hypot(truths[:, 0] - d.position[0], truths[:, 1] - d.position[1])
                if dd.min() <= 5.0:
                    errs.append(float(dd.min()))
    e = np.array(errs)
    return {"detections": int(len(e)),
            "detection_error_median_m": float(np.median(e)) if len(e) else float("nan"),
            "detection_error_p90_m": float(np.percentile(e, 90)) if len(e) else float("nan")}


def imaging_stats(bag, agents, cam_names, t0, t1):
    per = defaultdict(lambda: {"px": [], "p": []})
    for n in agents:
        for m in bag.get(f"/agent_{n}/visibility_truth", []):
            t = p1.stamp(m.stamp)
            if t < t0 or t > t1 or not m.visible:
                continue
            name = cam_names[m.camera] if m.camera < len(cam_names) else str(m.camera)
            per[name]["px"].append(m.pixels)
            per[name]["p"].append(m.p_detect)
    total = sum(len(v["px"]) for v in per.values())
    out = {}
    for name, v in per.items():
        px = np.array(v["px"])
        out[name] = {
            "share": len(px) / total if total else float("nan"),
            "pixels_median": float(np.median(px)),
            "pixels_p10": float(np.percentile(px, 10)),
            "p_detect_mean": float(np.mean(v["p"])),
            "detect": float(np.mean((px >= JOHNSON[0]) & (px < JOHNSON[1]))),
            "recognise": float(np.mean((px >= JOHNSON[1]) & (px < JOHNSON[2]))),
            "identify": float(np.mean(px >= JOHNSON[2])),
        }
    return out


def zoom_stats(msgs, t0, t1):
    rows = [(p1.stamp(m.stamp), m.track_id, m.on_target) for m in msgs]
    rows = [r for r in rows if t0 <= r[0] <= t1]
    if len(rows) < 2:
        return {}
    on = np.mean([r[1] != 0 and r[2] for r in rows])
    dwells, looks = [], defaultdict(list)
    cur, since = 0, rows[0][0]
    for t, tid, _ in rows:
        if tid != cur:
            if cur:
                dwells.append(t - since)
                looks[cur].append((since, t))
            cur, since = tid, t
    if cur:
        dwells.append(rows[-1][0] - since)
        looks[cur].append((since, rows[-1][0]))
    revisits = [b[0] - a[1] for v in looks.values() for a, b in zip(v, v[1:])]
    return {"on_target_share": float(on), "tracks_looked_at": len(looks),
            "dwell_mean_s": float(np.mean(dwells)) if dwells else float("nan"),
            "revisit_mean_s": float(np.mean(revisits)) if revisits else float("nan")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    import yaml

    sc = yaml.safe_load(open(a.logdir / "scenario.yaml"))
    agents = [ag["id"] for ag in sc["agents"]]
    entities = [e["name"] for e in sc["entities"]]
    det = sc["detector"]
    imaging = det.get("model") == "imaging"
    profile = "high" if imaging else "low"
    bag = p1.read_bag(a.logdir / "coop_bag")
    gt = p1.ground_truth(bag.get("/sim/ground_truth", []))
    vis = p3.visibility_by_entity(bag, agents)
    t_all = np.concatenate([g for g, _ in vis.values()])
    t_end = float(t_all.max())
    t_land = landing_time(bag, agents)
    if t_land is not None:
        t_end = min(t_end, t_land)
    t0 = on_station(gt, sc, bag)
    if t0 is None:
        print("no agent reached its altitude: nothing to score")
        return 1
    radius = float(sc["planner"]["interest_radius_m"])
    frames = [m for m in bag.get("/device/overlay/truth", []) if t0 <= p1.stamp(m.stamp) <= t_end]
    ov, _ = p3.overlay(frames, vis, gt, radius)
    ov_all, rows_all = p3.overlay(frames, vis)
    samples = {"device": p1.track_samples(bag.get("/device/tracks", []))}
    for n in agents:
        samples[f"agent_{n}"] = p1.track_samples(bag.get(f"/baseline_agent_{n}/tracks", []))
    tracks = {k: p3.mot(v, gt, vis, t0, t_end) for k, v in samples.items()}
    ov["mota"] = tracks["device"]["mota"]
    ph4 = {"profile": profile, "agents": len(agents),
           "altitude_m": [float(ag["altitude_m"]) for ag in sc["agents"]],
           "nav_bias": det.get("nav_bias", {"horizontal_sigma_m": 0.0, "vertical_sigma_m": 0.0}),
           "window_s": [t0, t_end]}
    ph4.update(awareness(samples["device"], gt, entities, t0, t_end, radius))
    ph4.update(seen_share(bag, agents, gt, t0, t_end, radius))
    ph4.update(detection_errors(bag, agents, gt, entities, t0, t_end, 0.9))
    if imaging:
        ph4["imaging"] = imaging_stats(bag, agents, list(det["cameras"]), t0, t_end)
    if "gimbal" in sc:
        ph4["zoom"] = zoom_stats(bag.get(f"/agent_{sc['gimbal']['agent']}/gimbal/state", []), t0, t_end)

    alt = ", ".join(f"{h:.0f}" for h in ph4["altitude_m"])
    bias = ph4["nav_bias"]
    print(f"{profile} profile: {len(agents)} agent(s) at {alt} m, navigation bias "
          f"{bias['horizontal_sigma_m']:.2f} / {bias['vertical_sigma_m']:.2f} m; "
          f"scored {t_end - t0:.0f} s on station (t = {t0:.0f}-{t_end:.0f} s), {len(entities)} entities")
    print(f"  device view, within {radius:.0f} m of the device   [whole field of view]")

    def line(label, key, fmt, unit=""):
        v = ov.get(key, float("nan"))
        lim = p3.THRESHOLDS.get(key)
        flag = ""
        if lim is not None and a.check:
            okv = (v >= lim[1]) if lim[0] == ">=" else (v <= lim[1])
            flag = "  ok" if okv else f"  FAIL ({lim[0]} {lim[1]})"
        print(f"    {label:<46}{format(v, fmt):>10}  {unit:<3}[{format(ov_all.get(key, float('nan')), fmt)}]{flag}")

    line("see-through availability (planner)", "see_through_availability", ".3f")
    line("see-through coverage (hidden, agents see it)", "see_through_coverage", ".3f")
    print(f"      (entity-frames hidden from the device and seen by an agent: "
          f"{ov.get('entity_frames_see_through', 0)} near, {ov_all.get('entity_frames_see_through', 0)} in view)")
    line("pixel error, see-through, median", "pixel_error_median_px", ".1f", "px")
    line("pixel error, see-through, p90", "pixel_error_p90_px", ".1f", "px")
    line("world error, see-through, median", "world_error_median_m", ".2f", "m")
    line("data age, live, p90", "age_p90_s", ".2f", "s")
    print(f"  situational picture                          near device    whole map")
    print(f"    seen by an agent (truth)                     {ph4['seen_near']:>8.3f}  {ph4['seen_map']:>10.3f}")
    print(f"    awareness (live device track on it)          {ph4['awareness_near']:>8.3f}  {ph4['awareness_map']:>10.3f}")
    d = tracks["device"]
    print(f"  device tracks: MOTA {d['mota']:.3f}  MOTP {d['motp_m']:.2f} m  continuity {d['continuity']:.3f}"
          f"  ID switches {d['id_switches']}  false-track samples {d['false_track_samples']}")
    print(f"  detections: {ph4['detections']}, horizontal error median {ph4['detection_error_median_m']:.2f} m,"
          f" p90 {ph4['detection_error_p90_m']:.2f} m")
    for name, s in ph4.get("imaging", {}).items():
        print(f"  camera {name:<5} {100 * s['share']:5.1f} % of sightings: {s['pixels_median']:.0f} px median"
              f" (p10 {s['pixels_p10']:.0f}), P(detect) {s['p_detect_mean']:.2f}; detect / recognise / identify"
              f" {s['detect']:.2f} / {s['recognise']:.2f} / {s['identify']:.2f}")
    z = ph4.get("zoom")
    if z:
        print(f"  zoom: on a target {100 * z['on_target_share']:.0f} % of the time, {z['tracks_looked_at']} tracks,"
              f" dwell {z['dwell_mean_s']:.1f} s, revisit every {z['revisit_mean_s']:.1f} s")

    json.dump({"overlay": ov, "overlay_whole_view": ov_all, "interest_radius_m": radius, "tracks": tracks,
               "phase4": ph4, "thresholds": {k: list(v) for k, v in p3.THRESHOLDS.items()}},
              open(a.logdir / "metrics.json", "w"), indent=2, default=float)
    p3.plot(a.logdir, gt, bag, agents, sc["world_file"], [e["model"] for e in sc["entities"]],
            p3.pick_frames(a.logdir / "frames", rows_all), entities, name="phase4")
    failed = []
    for k, (op, lim) in p3.THRESHOLDS.items():
        v = ov.get(k, float("nan"))
        if v != v or not ((v >= lim) if op == ">=" else (v <= lim)):
            failed.append(k)
    if a.check and failed:
        print(f"FAILED thresholds: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
