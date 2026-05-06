#!/usr/bin/env python3
"""parse.py — ArduPilot/MAVLink flight log → JSON

CLI:
    python parse.py mylog.bin                  # writes mylog.bin.json next to it
    python parse.py mylog.bin -o out.json
    python parse.py mylog.bin -o -             # stdout
    python parse.py mylog.bin --pretty         # indented JSON

Library:
    from parse import parse_log
    data = parse_log(Path("mylog.bin"))

Handles ArduPilot DataFlash (.bin), MAVLink-with-timestamps (.tlog), and the
text variant (.log). Emits a single JSON blob the Flask viewer consumes
directly.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil


EARTH_R_M = 6_371_000.0

# Forensic anomaly thresholds. A waypoint farther than this from the rest of
# the track is almost certainly leftover, a coordinate-typo, or planted
# misdirection — exactly the kind of artifact the workshop is looking for.
WAYPOINT_OUTLIER_KM = 50.0

# Real ArduPilot logs run 5-25 Hz GPS. Anything looser is suspicious.
GPS_GAP_S = 1.0
GPS_JUMP_M = 50.0
ALT_JUMP_M = 10.0

# ArduCopter mode numbers. Plane/Rover use different tables; we bias to
# copter because that's what ArduPilot forensics workshops fly.
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
    17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
    21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
}

# Subset of ArduPilot EV ids worth resolving to names. Full table lives in
# ardupilot/libraries/AP_Logger/LogStructure.h — extend as needed.
EV_NAMES = {
    7: "AP_STATE", 8: "SYSTEM_TIME_SET", 9: "INIT_SIMPLE_BEARING",
    10: "ARMED", 11: "DISARMED", 15: "AUTO_PAUSED_OR_RESUMED",
    16: "SET_HOME", 17: "LAND", 18: "LAND_COMPLETE",
    25: "AUTO_ARMED", 28: "TAKEOFF", 29: "LAND_COMPLETE_MAYBE",
    30: "PARACHUTE_RELEASED", 41: "NOT_LANDED", 42: "LOST_GPS",
    54: "ZIGZAG_STORE_LEG", 57: "STANDBY_ENABLE", 58: "STANDBY_DISABLE",
}

# MAV_CMD constants we care to label. NAV_WAYPOINT(16), NAV_TAKEOFF(22),
# NAV_RTL(20), NAV_LAND(21), NAV_LOITER_TIME(19) cover almost all real missions.
CMD_NAMES = {
    16: "WAYPOINT", 17: "LOITER_UNLIM", 18: "LOITER_TURNS",
    19: "LOITER_TIME", 20: "RTL", 21: "LAND", 22: "TAKEOFF",
    82: "SPLINE_WAYPOINT", 84: "VTOL_TAKEOFF", 85: "VTOL_LAND",
    93: "DELAY", 177: "DO_JUMP", 178: "DO_CHANGE_SPEED",
    179: "DO_SET_HOME", 201: "DO_SET_ROI",
}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R_M * math.asin(math.sqrt(a))


def msg_time(m: Any) -> float | None:
    """Canonical seconds-clock for one message.

    Two time bases are possible and we never mix them:
      - DataFlash records have TimeUS (microseconds since boot)
      - tlog records have _timestamp already in unix seconds
    Returns boot-seconds for DataFlash, unix-seconds for tlog, None otherwise.
    Falling back to _timestamp on a DataFlash record without TimeUS would
    silently inject a 1.7e9-sec value into a 100-sec timeline.
    """
    if hasattr(m, "TimeUS"):
        return m.TimeUS / 1e6
    ts = getattr(m, "_timestamp", None)
    if ts and ts > 1_000_000_000:
        return float(ts)
    return None


def gps_coords(m: Any) -> tuple[float, float, float] | None:
    """Pull (lat, lon, alt_m) from any GPS-bearing message; None if not GPS
    or pre-fix or contains NaN/zero placeholder."""
    t = m.get_type()
    if t == "GPS":
        # DataFlash GPS: degrees + meters already. Drop pre-fix samples.
        if hasattr(m, "Status") and int(m.Status) < 3:
            return None
        lat = float(getattr(m, "Lat", 0.0))
        lon = float(getattr(m, "Lng", 0.0))
        alt = float(getattr(m, "Alt", 0.0))
    elif t in ("GPS_RAW_INT", "GPS2_RAW"):
        if hasattr(m, "fix_type") and int(m.fix_type) < 3:
            return None
        lat = m.lat / 1e7
        lon = m.lon / 1e7
        alt = m.alt / 1000.0
    elif t == "GLOBAL_POSITION_INT":
        lat = m.lat / 1e7
        lon = m.lon / 1e7
        alt = m.alt / 1000.0
    else:
        return None
    if lat == 0.0 and lon == 0.0:
        return None
    if math.isnan(lat) or math.isnan(lon) or math.isnan(alt):
        return None
    return lat, lon, alt


def event_payload(m: Any) -> tuple[int | None, str] | None:
    """(severity, message) for any text-bearing message, else None."""
    t = m.get_type()
    if t == "STATUSTEXT":
        text = (m.text or "").strip()
        return int(m.severity), text
    if t == "MSG":
        # DataFlash text — field name varies between firmware versions.
        text = getattr(m, "Message", None) or getattr(m, "message", None) or ""
        return None, text.strip()
    if t == "ERR":
        return None, f"Subsys={int(m.Subsys)} ECode={int(m.ECode)}"
    if t == "EV":
        eid = int(m.Id)
        return None, f"{EV_NAMES.get(eid, 'EV')} (id={eid})"
    return None


def waypoint_data(m: Any) -> dict[str, Any] | None:
    t = m.get_type()
    if t == "CMD":
        cid = int(getattr(m, "CId", 0))
        return {
            "seq": int(getattr(m, "CNum", 0)),
            "cmd": cid,
            "cmd_name": CMD_NAMES.get(cid, f"CMD_{cid}"),
            "lat": float(getattr(m, "Lat", 0.0)),
            "lon": float(getattr(m, "Lng", 0.0)),
            "alt": float(getattr(m, "Alt", 0.0)),
        }
    if t in ("MISSION_ITEM", "MISSION_ITEM_INT"):
        scale = 1e7 if t == "MISSION_ITEM_INT" else 1.0
        cid = int(m.command)
        return {
            "seq": int(m.seq),
            "cmd": cid,
            "cmd_name": CMD_NAMES.get(cid, f"CMD_{cid}"),
            "lat": float(m.x) / scale,
            "lon": float(m.y) / scale,
            "alt": float(m.z),
        }
    return None


def battery_data(m: Any) -> dict[str, float] | None:
    """{voltage:V, current:A} or None. Filters NaN — ArduCopter 4.0.x leaves
    BAT.EnrgTot NaN, so we read Volt/Curr directly and ignore derived fields."""
    t = m.get_type()
    if t == "BAT":
        v = float(getattr(m, "Volt", 0.0))
        c = float(getattr(m, "Curr", 0.0))
    elif t == "CURR":
        # Legacy: Volt/Curr in centi-units (V*100, A*100).
        v = float(getattr(m, "Volt", 0.0)) / 100.0
        c = float(getattr(m, "Curr", 0.0)) / 100.0
    elif t == "BATTERY_STATUS":
        vs = m.voltages
        v = (vs[0] / 1000.0) if vs and vs[0] != 0xFFFF else 0.0
        c = (m.current_battery / 100.0) if m.current_battery >= 0 else 0.0
    else:
        return None
    if math.isnan(v) or math.isnan(c):
        return None
    return {"voltage": round(v, 3), "current": round(c, 3)}


def mode_data(m: Any) -> dict[str, Any] | None:
    t = m.get_type()
    if t == "MODE":
        num = getattr(m, "Mode", None)
        if num is None:
            num = getattr(m, "ModeNum", None)
        num = int(num) if num is not None else -1
        name = str(getattr(m, "ModeName", "")) or COPTER_MODES.get(num)
        return {"num": num, "name": name}
    if t == "HEARTBEAT" and hasattr(m, "custom_mode"):
        num = int(m.custom_mode)
        return {"num": num, "name": COPTER_MODES.get(num)}
    return None


def detect_format(path: Path) -> str:
    return {
        ".bin": "dataflash",
        ".tlog": "mavlink",
        ".log": "text",
    }.get(path.suffix.lower(), "unknown")


def boot_banner(events: list[dict]) -> str | None:
    """Stitch firmware identifier from first ~30 MSG events.

    ArduPilot fires a boot banner as a series of MSG rows on startup
    (firmware version, HAL, board fingerprint, frame, GPS, RC protocol).
    We collect the identifying lines and join them with a separator.
    """
    keywords = (
        "ArduCopter", "ArduPlane", "ArduSub", "ArduRover", "ArduTracker",
        "ChibiOS", "PixHawk", "Pixhawk", "CubeOrange", "CubeSolo",
        "Frame:", "u-blox", "RC Protocol",
    )
    keep: list[str] = []
    seen: set[str] = set()
    for ev in events[:30]:
        if ev.get("type") != "MSG":
            continue
        msg = (ev.get("message") or "").strip()
        if not msg or msg in seen:
            continue
        if any(kw in msg for kw in keywords):
            keep.append(msg)
            seen.add(msg)
        if len(keep) >= 5:
            break
    return " · ".join(keep) or None


def detect_anomalies(track: list[dict], battery: list[dict]) -> list[dict]:
    """Forensic anomalies derived from the track and battery streams."""
    out: list[dict] = []
    for a, b in zip(track, track[1:]):
        dt = b["t"] - a["t"]
        if dt > GPS_GAP_S:
            out.append({
                "t": a["t"],
                "kind": "gps_gap",
                "detail": f"{dt:.2f}s gap between consecutive GPS samples",
                "lat": a["lat"], "lon": a["lon"],
            })
        d = haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])
        if d > GPS_JUMP_M:
            speed = d / max(dt, 1e-3)
            out.append({
                "t": b["t"],
                "kind": "gps_jump",
                "detail": f"{d:.0f}m position jump (~{speed:.0f} m/s)",
                "lat": b["lat"], "lon": b["lon"],
            })
        if abs(b["alt"] - a["alt"]) > ALT_JUMP_M:
            out.append({
                "t": b["t"],
                "kind": "alt_jump",
                "detail": f"{abs(b['alt']-a['alt']):.1f}m altitude change in {dt:.2f}s",
                "lat": b["lat"], "lon": b["lon"],
            })
    for bat in battery:
        if bat["voltage"] > 100.0 or bat["voltage"] < 0:
            out.append({
                "t": bat["t"],
                "kind": "battery_anomaly",
                "detail": f"voltage {bat['voltage']:.1f}V outside plausible range",
                "lat": None, "lon": None,
            })
    return out


def parse_log(path: Path) -> dict[str, Any]:
    """Single pass: collect with absolute timestamps, then normalize against
    the first GPS-locked sample. Two-pass keeps the inner loop simple and
    keeps timestamps consistent regardless of which message arrives first."""
    if not path.exists():
        raise FileNotFoundError(f"no such file: {path}")

    mlog = mavutil.mavlink_connection(str(path), dialect="ardupilotmega")

    track: list[dict] = []
    modes: list[dict] = []
    events: list[dict] = []
    waypoints: list[dict] = []
    battery: list[dict] = []
    home: dict[str, Any] | None = None
    home_source: str | None = None
    bad_data = 0

    last_lat = last_lon = last_t = None
    # (TimeUS-secs, unix-epoch) pair from the first message we can pin to the
    # wall clock. Two possible sources:
    #   - DataFlash GPS row carries true GPS week + ms-of-week
    #   - tlog _timestamp is already unix microseconds (>1e9 when set)
    # Pre-fix DataFlash records have _timestamp == TimeUS/1e6, which looks
    # like a 1970 timestamp — explicitly rejected by the >1e9 gate below.
    wall_anchor: tuple[float, float] | None = None

    while True:
        try:
            m = mlog.recv_match(blocking=False)
        except Exception as e:
            print(f"[parse] frame error: {e}", file=sys.stderr)
            continue
        if m is None:
            break
        if m.get_type() == "BAD_DATA":
            bad_data += 1
            continue

        t_abs = msg_time(m)
        if t_abs is None:
            continue

        mt = m.get_type()

        coords = gps_coords(m)
        if coords is not None:
            lat, lon, alt = coords

            # Anchor wall-clock from first GPS-locked sample only. Doing this
            # in the GPS branch guarantees t_abs and wall_anchor[0] share a
            # time base — boot-secs for DataFlash, unix for tlog.
            if wall_anchor is None:
                gw = int(getattr(m, "GWk", 0) or 0)
                if gw > 0:  # DataFlash GPS carries true GPS week+ms.
                    gms = int(getattr(m, "GMS", 0) or 0)
                    # GPS epoch 1980-01-06; leap-second offset is 18 in 2026.
                    unix_t = 315_964_800 + gw * 7 * 86400 + gms / 1000.0 - 18.0
                    wall_anchor = (t_abs, unix_t)
                else:
                    ts_wall = getattr(m, "_timestamp", None)
                    if ts_wall and ts_wall > 1_000_000_000:
                        wall_anchor = (t_abs, float(ts_wall))

            spd = getattr(m, "Spd", None)
            if spd is None and last_t is not None and t_abs > last_t:
                spd = haversine_m(last_lat, last_lon, lat, lon) / (t_abs - last_t)
            track.append({
                "_t_abs": t_abs,
                "lat": lat, "lon": lon,
                "alt": round(alt, 2),
                "spd": round(float(spd), 3) if spd is not None else None,
            })
            last_lat, last_lon, last_t = lat, lon, t_abs
            continue

        if mt in ("HOME", "ORGN") and home is None:
            lat = float(getattr(m, "Lat", 0.0))
            lon = float(getattr(m, "Lng", 0.0))
            alt = float(getattr(m, "Alt", 0.0))
            if not (lat == 0.0 and lon == 0.0):
                home = {"lat": lat, "lon": lon, "alt": alt}
                home_source = mt
            continue

        md = mode_data(m)
        if md is not None:
            md["_t_abs"] = t_abs
            modes.append(md)
            continue

        ev = event_payload(m)
        if ev is not None:
            sev, text = ev
            if not text:
                continue
            events.append({
                "_t_abs": t_abs,
                "type": mt,
                "severity": sev,
                "message": text,
            })
            continue

        wp = waypoint_data(m)
        if wp is not None:
            waypoints.append(wp)
            continue

        bd = battery_data(m)
        if bd is not None:
            bd["_t_abs"] = t_abs
            battery.append(bd)
            continue

    # --- normalize timestamps against first GPS-locked sample ---
    if track:
        t0 = track[0]["_t_abs"]
    elif modes:
        t0 = modes[0]["_t_abs"]
    elif events:
        t0 = events[0]["_t_abs"]
    else:
        t0 = 0.0

    def _norm(lst: list[dict]) -> None:
        for r in lst:
            ta = r.pop("_t_abs", None)
            r["t"] = round(ta - t0, 3) if ta is not None else None

    _norm(track)
    _norm(modes)
    _norm(events)
    _norm(battery)

    # Home fallback: HOME → ORGN → first GPS sample
    if home is None and track:
        home = {"lat": track[0]["lat"], "lon": track[0]["lon"], "alt": track[0]["alt"]}
        home_source = "first_gps"
    if home is not None:
        home["source"] = home_source

    # Wall-clock for the log's t=0 (first GPS-locked sample).
    started_at: str | None = None
    if wall_anchor is not None:
        offset = wall_anchor[0] - t0
        ts0 = wall_anchor[1] - offset
        # Sanity gate: anything pre-Y2K means our anchor was bogus.
        if ts0 > 946_684_800:
            try:
                started_at = (
                    datetime.fromtimestamp(ts0, tz=timezone.utc)
                    .isoformat()
                    .replace("+00:00", "Z")
                )
            except (OSError, ValueError):
                pass

    firmware = boot_banner(events)
    anomalies = detect_anomalies(track, battery)

    # Waypoint outlier flag — distance from track centroid. Skip 0/0
    # placeholders (TAKEOFF/RTL legitimately use them).
    cent: tuple[float, float] | None = None
    if track:
        n = len(track)
        cent = (sum(p["lat"] for p in track) / n, sum(p["lon"] for p in track) / n)
    for wp in waypoints:
        wp["outlier"] = False
        if cent is None:
            continue
        if wp["lat"] == 0.0 and wp["lon"] == 0.0:
            continue
        d_km = haversine_m(cent[0], cent[1], wp["lat"], wp["lon"]) / 1000.0
        if d_km > WAYPOINT_OUTLIER_KM:
            wp["outlier"] = True
            anomalies.append({
                "t": None,
                "kind": "waypoint_outlier",
                "detail": f"WP{wp['seq']} ({wp['cmd_name']}) is {d_km:,.0f} km from track",
                "lat": wp["lat"], "lon": wp["lon"],
            })

    duration_s = round(track[-1]["t"], 3) if track else 0.0
    max_alt = round(max((p["alt"] for p in track), default=0.0), 2)
    max_spd = round(max((p["spd"] or 0.0 for p in track), default=0.0), 3)
    total_dist_m = round(
        sum(haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])
            for a, b in zip(track, track[1:])),
        2,
    )
    # If anything moved meaningfully, the vehicle was armed at some point.
    armed = max_spd > 1.0

    return {
        "filename": path.name,
        "format": detect_format(path),
        "firmware": firmware,
        "started_at": started_at,
        "duration_s": duration_s,
        "home": home,
        "stats": {
            "gps_points": len(track),
            "max_alt": max_alt,
            "max_speed": max_spd,
            "total_distance_m": total_dist_m,
            "events": len(events),
            "anomalies": len(anomalies),
            "duration_s": duration_s,
            "armed": armed,
            "bad_data_frames": bad_data,
        },
        "track": track,
        "modes": modes,
        "events": events,
        "waypoints": waypoints,
        "battery": battery,
        "anomalies": anomalies,
    }


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Parse ArduPilot/MAVLink log → JSON")
    ap.add_argument("log", type=Path)
    ap.add_argument("-o", "--out", default=None,
                    help="output path (default: <log>.json), '-' for stdout")
    ap.add_argument("--pretty", action="store_true",
                    help="pretty-print JSON (default: compact)")
    args = ap.parse_args(argv)

    data = parse_log(args.log)
    text = json.dumps(data, indent=2 if args.pretty else None, default=str)

    if args.out == "-":
        sys.stdout.write(text)
    else:
        out = Path(args.out) if args.out else args.log.with_suffix(args.log.suffix + ".json")
        out.write_text(text)
        print(f"[parse] wrote {out} ({len(text):,} bytes)", file=sys.stderr)

    s = data["stats"]
    armed = "ARMED" if s["armed"] else "ground"
    fw = (data.get("firmware") or "unknown").split(" · ")[0]
    print(
        f"[parse] {s['gps_points']} pts · {s['duration_s']}s · max {s['max_alt']}m "
        f"· {s['total_distance_m']}m traveled · {s['events']} events "
        f"· {s['anomalies']} anomalies · {armed} · {fw}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
