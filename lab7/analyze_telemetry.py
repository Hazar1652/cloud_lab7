#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path


def _f(row, *names):
    for name in names:
        if name in row and row[name] not in ("", None):
            try:
                return float(row[name])
            except ValueError:
                continue
    return None


def load_csv(path):
    with open(path, newline="", encoding="utf-8", errors="replace") as fh:
        sample = fh.read(4096)
        fh.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
        except csv.Error:
            dialect = csv.excel
        rows = list(csv.DictReader(fh, dialect=dialect))
    return rows


def pick_time(row):
    t = _f(row, "timestamp", "time_boot_ms", "T", "time")
    if t is None:
        return None
    if t > 1e12:
        t = t / 1000.0
    elif t > 1e10:
        t = t / 1000.0
    return t


def relative_alt(row):
    alt = _f(
        row,
        "GLOBAL_POSITION_INT.relative_alt",
        "relative_alt",
        "vehicle.altitudeRelative",
        "CTUN.Alt",
        "z",
    )
    if alt is None:
        return None
    if abs(alt) > 50:
        alt = alt / 1000.0
    return alt


def ground_speed(row):
    hud = _f(row, "VFR_HUD.groundspeed", "groundspeed", "GPS.Spd", "hspeed")
    if hud is not None:
        return abs(hud) / 100.0 if abs(hud) > 50 else abs(hud)
    vx = _f(row, "GLOBAL_POSITION_INT.vx", "vx")
    vy = _f(row, "GLOBAL_POSITION_INT.vy", "vy")
    if vx is None or vy is None:
        return None
    return math.hypot(vx, vy) / 100.0


def lat_lon(row):
    lat = _f(row, "GLOBAL_POSITION_INT.lat", "lat", "latitude")
    lon = _f(row, "GLOBAL_POSITION_INT.lon", "lon", "longitude")
    if lat is None or lon is None:
        return None, None
    if abs(lat) > 180:
        lat = lat / 1e7
    if abs(lon) > 180:
        lon = lon / 1e7
    return lat, lon


def roll_deg(row):
    roll = _f(row, "ATTITUDE.roll", "roll", "ATT.Roll")
    if roll is None:
        return None
    return math.degrees(roll) if abs(roll) <= math.pi + 0.1 else roll


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_telemetry.py <telemetry.csv>")
        sys.exit(1)

    path = Path(sys.argv[1])
    rows = load_csv(path)
    if not rows:
        print("Empty CSV")
        sys.exit(1)

    samples = []
    for row in rows:
        samples.append(
            {
                "t": pick_time(row),
                "alt": relative_alt(row),
                "spd": ground_speed(row),
                "roll": roll_deg(row),
                "lat": lat_lon(row)[0],
                "lon": lat_lon(row)[1],
            }
        )

    airborne = [s for s in samples if s["alt"] is not None and s["alt"] > 1.0 and s["t"] is not None]
    segment = []
    best = []
    for s in airborne:
        if segment and s["t"] - segment[-1]["t"] > 15.0:
            if len(segment) > len(best):
                best = segment
            segment = [s]
        else:
            segment.append(s)
    if len(segment) > len(best):
        best = segment
    window = best or airborne or samples
    t0 = min((s["t"] for s in window if s["t"] is not None), default=None)
    t1 = max((s["t"] for s in window if s["t"] is not None), default=None)

    def in_window(s):
        if t0 is None or s["t"] is None:
            return True
        return t0 <= s["t"] <= t1

    alts = [s["alt"] for s in window if s["alt"] is not None]
    speeds = [s["spd"] for s in window if s["spd"] is not None]
    rolls = [s["roll"] for s in samples if s["roll"] is not None and in_window(s)]
    coords = [(s["lat"], s["lon"]) for s in samples if s["lat"] is not None]
    air_times = [s["t"] for s in window if s["t"] is not None]

    target = (-35.362922, 149.164207)
    start = coords[0] if coords else (None, None)
    end = coords[-1] if coords else (None, None)
    err_m = None
    if end[0] is not None:
        dlat = (end[0] - target[0]) * 111320.0
        dlon = (end[1] - target[1]) * 111320.0 * math.cos(math.radians(target[0]))
        err_m = math.hypot(dlat, dlon)

    duration = (max(air_times) - min(air_times)) if len(air_times) >= 2 else None

    print("----- Таблиця 7.4 -----")
    print(f"Найбільша висота:     {max(alts):.2f} м" if alts else "Найбільша висота: n/a")
    print(f"Найбільша швидкість:  {max(speeds):.3f} м/с" if speeds else "Найбільша швидкість: n/a")
    print(f"Час польоту:          {duration:.1f} с" if duration else "Час польоту: n/a")
    if start[0] is not None:
        print(f"Початкова координата: {start[0]:.6f}, {start[1]:.6f}")
        print(f"Кінцева координата:   {end[0]:.6f}, {end[1]:.6f}")
    print(f"Максимальний |roll|:  {max(abs(r) for r in rolls):.2f}°" if rolls else "Максимальний |roll|: n/a")
    if err_m is not None:
        print(f"Відстань до WP:       {err_m:.2f} м  (ціль {target[0]}, {target[1]})")


if __name__ == "__main__":
    main()
