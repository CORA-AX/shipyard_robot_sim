"""Robot-independent body velocity command and wall-time expiry."""
import math
import time


class VelocityCommand:
    """Body-frame vx/vy/yaw command; stale or invalid input means stand."""
    def __init__(self, timeout=0.5, clock=time.monotonic):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Command timeout must be finite and positive.')
        self.timeout = timeout
        self.clock = clock
        self.values = (0.0, 0.0, 0.0)
        self.received_at = None
        self.received = self.rejected = self.clamped = 0

    def update(self, vx, vy, wz):
        values = (vx, vy, wz)
        if not all(math.isfinite(value) for value in values):
            self.values = (0.0, 0.0, 0.0)
            self.received_at = None
            self.rejected += 1
            return False
        self.values = values
        self.received_at = self.clock()
        self.received += 1
        return True

    @property
    def stale(self):
        return self.received_at is None or self.clock() - self.received_at >= self.timeout

    def clear(self):
        """Invalidate a pre-reset command without changing transport counters."""
        self.values = (0., 0., 0.)
        self.received_at = None

    def current(self):
        return (0.0, 0.0, 0.0) if self.stale else self.values

