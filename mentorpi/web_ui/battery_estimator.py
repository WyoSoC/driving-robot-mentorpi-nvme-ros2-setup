"""Battery state for the MentorPi web UI: charge %, charging/discharging, time estimates.

The RRC Lite board reports only the pack voltage (~1 Hz, +-5 mV noise): no current and no
charger flag. So everything here is inferred from the voltage trend:

- charge % from a typical Li-ion/LiPo resting-voltage curve, 2 cells (0% = 6.4 V, Hiwonder's
  "charge below 6.4 V"; 100% = 8.4 V)
- readings are reduced to 30 s medians; periods while the car drives are skipped (the motors
  make the voltage sag)
- the trend is a least-squares slope of charge % over the last few minutes:
  rising -> charging (only possible with the car left on while charging),
  falling -> in use, flat -> steady
- time to empty / to full = remaining % / rate. Rough: while charging the voltage reads high
  (charger pushes it up) and the last ~15 min are a constant-voltage top-off the voltage
  can't show, so a fixed allowance is added.

Pure Python (no ROS) so it can be tested off the robot: python3 battery_estimator.py
"""
import statistics

CELLS = 2
# Resting voltage per cell -> charge %, typical Li-ion/LiPo curve
OCV = [(3.20, 0), (3.30, 2), (3.45, 5), (3.60, 10), (3.68, 15), (3.72, 20), (3.74, 25),
       (3.76, 30), (3.78, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.86, 55), (3.89, 60),
       (3.92, 65), (3.96, 70), (4.00, 75), (4.04, 80), (4.08, 85), (4.12, 90), (4.16, 95),
       (4.20, 100)]
LOW_V = 6.9           # RRC Lite low-battery alarm (battery_level 0x1af4 = 6900 mV)
CRITICAL_V = 6.6      # close to Hiwonder's 6.4 V "charge now"
BIN_S = 30            # median bins
WINDOW_S = 10 * 60    # trend window
MIN_SPAN_S = 4 * 60   # need this much (non-driving) data for a trend
DRIVE_HOLDOFF_S = 20  # ignore readings until this long after driving stopped
CHARGING_RATE = 0.2   # %/min; above -> charging
USING_RATE = -0.2     # %/min; below -> in use
FULL_V = 8.33         # at/above while not falling -> full or held by the charger
TOPOFF_MIN = 15       # constant-voltage top-off the voltage can't show


def soc_from_voltage(volts):
    """Charge % (0-100) from pack voltage."""
    v = volts / CELLS
    if v <= OCV[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(OCV, OCV[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0


class BatteryEstimator:
    def __init__(self):
        self.raw = []          # (t, volts) of the current bin
        self.bins = []         # (t_mid, median volts)
        self.last_drive = -1e9
        self.last_t = None

    def driving(self, t):
        self.last_drive = t

    def add(self, t, volts):
        self.last_t = t
        if t - self.last_drive < DRIVE_HOLDOFF_S:
            self.raw = []      # sagging: drop the partial bin
            return
        self.raw.append((t, volts))
        if self.raw[-1][0] - self.raw[0][0] >= BIN_S:
            ts = [x[0] for x in self.raw]
            self.bins.append(((ts[0] + ts[-1]) / 2, statistics.median(x[1] for x in self.raw)))
            self.raw = []
        self.bins = [b for b in self.bins if t - b[0] <= WINDOW_S]

    def status(self, volts):
        soc = soc_from_voltage(volts)
        out = {"v": round(volts, 3), "soc": round(soc), "state": "collecting", "rate": None,
               "eta_min": None, "eta_kind": None,
               "level": "critical" if volts < CRITICAL_V else "low" if volts < LOW_V else "ok"}
        b = self.bins
        span = b[-1][0] - b[0][0] if len(b) >= 2 else 0
        if len(b) < 5 or span < MIN_SPAN_S:
            out["collecting_s"] = round(max(0, MIN_SPAN_S - span))
            return out
        xs = [(x[0] - b[0][0]) / 60 for x in b]            # minutes
        ys = [soc_from_voltage(x[1]) for x in b]
        mx, my = statistics.fmean(xs), statistics.fmean(ys)
        rate = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        out["rate"] = round(rate, 3)                         # %/min
        if volts >= FULL_V and rate > USING_RATE:
            out["state"] = "full"
        elif rate > CHARGING_RATE:
            out["state"] = "charging"
            out["eta_min"] = round((100 - soc) / rate + TOPOFF_MIN)
            out["eta_kind"] = "full"
        elif rate < USING_RATE:
            out["state"] = "discharging"
            out["eta_min"] = round(soc / -rate)
            out["eta_kind"] = "empty"
        else:
            out["state"] = "steady"
        return out


def _simulate(name, volts_at, minutes, drive=None):
    """Feed a synthetic voltage curve at 1 Hz; return the final status."""
    import random
    random.seed(1)
    e = BatteryEstimator()
    for s in range(int(minutes * 60)):
        t = float(s)
        if drive and drive[0] <= s / 60 < drive[1]:
            e.driving(t)
            v = volts_at(s / 60) - 0.25                       # sag while driving
        else:
            v = volts_at(s / 60)
        e.add(t, v + random.gauss(0, 0.005))
    st = e.status(volts_at(minutes))
    print(f"{name:42s} {st}")
    return st


if __name__ == "__main__":
    # car 2 today: 8.35 V -> 7.17 V in ~75 min while idle (Pi + sensors)
    d = _simulate("discharging (car 2 curve), 12 min", lambda m: 8.35 - (8.35 - 7.17) * m / 75, 12)
    assert d["state"] == "discharging" and d["eta_kind"] == "empty"
    d2 = _simulate("discharging with 2 min of driving", lambda m: 8.35 - (8.35 - 7.17) * m / 75, 12, (5, 7))
    assert d2["state"] == "discharging"
    c = _simulate("charging (car on), 7.2 -> 8.2 V over 60 min", lambda m: 7.2 + 1.0 * m / 60, 15)
    assert c["state"] == "charging" and c["eta_kind"] == "full"
    f = _simulate("held at 8.40 V by charger", lambda m: 8.40, 10)
    assert f["state"] == "full"
    s = _simulate("steady 8.25 V (bench supply)", lambda m: 8.25, 10)
    assert s["state"] == "steady"
    n = _simulate("just started (2 min)", lambda m: 8.0, 2)
    assert n["state"] == "collecting"
    low = _simulate("low battery 6.8 V, discharging", lambda m: 7.0 - 0.02 * m, 10)
    assert low["level"] == "low"
    print("all checks passed")
