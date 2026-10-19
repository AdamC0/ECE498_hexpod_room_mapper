"""Require observations and localization after every stop before moving again."""
import math


class StationarySurvey:
    def __init__(self, duration=1.0, scans=5):
        if not math.isfinite(duration) or duration <= 0 or scans < 1:
            raise ValueError('survey duration and scan count must be positive')
        self.duration, self.required = duration, scans
        self.reset(0.0)

    def reset(self, settled_at):
        self.settled_at = settled_at
        self.count = 0
        self.last_stamp = self.required_stamp = self.pose_stamp = 0.0

    def observe(self, stamp):
        if math.isfinite(stamp) and stamp > self.last_stamp:
            self.last_stamp = stamp
            self.count += 1
            if self.count == self.required:
                self.required_stamp = stamp

    def localize(self, stamp):
        if math.isfinite(stamp):
            self.pose_stamp = max(self.pose_stamp, stamp)

    def ready(self, now):
        return (now >= self.settled_at + self.duration and
                self.count >= self.required and
                self.pose_stamp >= self.required_stamp)
