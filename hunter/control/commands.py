"""Hunter speed/virtual-steering commands and wall-time expiry."""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class DriveCommand:
    speed: float = 0.
    steering_angle: float = 0.
    steering_angle_velocity: float = 0.
    acceleration: float = 0.
    jerk: float = 0.

    def valid(self):
        return (all(math.isfinite(v) for v in (self.speed, self.steering_angle,
                    self.steering_angle_velocity, self.acceleration, self.jerk))
                and self.steering_angle_velocity >= 0. and self.acceleration >= 0. and self.jerk == 0.)


class CommandBuffer:
    """Expiry stops propulsion; the controller holds steering while braking.

    Nonzero jerk is not supported in this stage and is rejected, not ignored.
    Zero acceleration/steering rate means use the configured vehicle limits.
    """
    def __init__(self, timeout=0.5, clock=time.monotonic):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Command timeout must be finite and positive.')
        self.timeout = timeout
        self.clock = clock
        self.values = DriveCommand()
        self.received_at = None
        self.received = self.rejected = 0

    def update(self, command):
        if not command.valid():
            self.values = DriveCommand()
            self.received_at = None
            self.rejected += 1
            return False
        self.values = command
        self.received_at = self.clock()
        self.received += 1
        return True

    @property
    def stale(self):
        return self.received_at is None or self.clock() - self.received_at >= self.timeout

    def clear(self):
        """Invalidate a pre-reset command without changing transport counters."""
        self.values = DriveCommand()
        self.received_at = None

    def current(self):
        return DriveCommand(steering_angle=self.values.steering_angle) if self.stale else self.values

