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


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_telemetry.py <telemetry.csv>")
        sys.exit(1)

    path = Path(sys.argv[1])
    rows = load_csv(path)
    if not rows:
        print("Empty CSV")
        sys.exit(1)

    alts, speeds, rolls, lats, lons, times = [], [], [], [], [], []
    for row in rows:
        alt = _f(
            row,
            "relative_alt",
            "alt",
            "z",
            "GLOBAL_POSITION_INT.relative_alt",
            "vehicle.altitudeRelative",
        )
        if alt is not None and abs(alt) > 50:
            alt = alt / 1000.0
        spd = _f(
            row,
            "groundspeed",
            "hspeed",
            "vx",
            "GLOBAL_POSITION_INT.vx",
        )
        roll = _f(row, "roll", "ATTITUDE.roll")
        lat = _f(row, "lat", "latitude", "GLOBAL_POSITION_INT.lat")
        lon = _f(row, "lon", "longitude", "GLOBAL_POSITION_INT.lon")
        if lat is not None and abs(lat) > 180:
            lat = lat / 1e7
        if lon is not None and abs(lon) > 180:
            lon = lon / 1e7
        t = pick_time(row)
        if alt is not None:
            alts.append(alt)
        if spd is not None:
            speeds.append(abs(spd) if abs(spd) < 50 else abs(spd) / 100.0)
        if roll is not None:
            rolls.append(math.degrees(roll) if abs(roll) <= math.pi + 0.1 else roll)
        if lat is not None and lon is not None:
            lats.append(lat)
            lons.append(lon)
        if t is not None:
            times.append(t)

    target = (-35.362922, 149.164207)
    end = (lats[-1], lons[-1]) if lats else (None, None)
    err_m = None
    if end[0] is not None:
        dlat = (end[0] - target[0]) * 111320.0
        dlon = (end[1] - target[1]) * 111320.0 * math.cos(math.radians(target[0]))
        err_m = math.hypot(dlat, dlon)

    duration = (max(times) - min(times)) if len(times) >= 2 else None

    print("----- Table 7.4 -----")
    print(f"Max altitude:     {max(alts):.2f} m" if alts else "Max altitude: n/a")
    print(f"Max speed:        {max(speeds):.2f} m/s" if speeds else "Max speed: n/a")
    print(f"Flight time:      {duration:.1f} s" if duration else "Flight time: n/a")
    if lats:
        print(f"Start:            {lats[0]:.6f}, {lons[0]:.6f}")
        print(f"End:              {lats[-1]:.6f}, {lons[-1]:.6f}")
    print(f"Max |roll|:       {max(abs(r) for r in rolls):.2f} deg" if rolls else "Max roll: n/a")
    if err_m is not None:
        print(f"Distance to WP:   {err_m:.2f} m  (target {target[0]}, {target[1]})")


if __name__ == "__main__":
    main()
