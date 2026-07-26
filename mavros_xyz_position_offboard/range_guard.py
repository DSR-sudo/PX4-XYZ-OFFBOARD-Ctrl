"""Range validity and recovery policy."""

from __future__ import annotations

from typing import Optional

try:
    from .core_types import RangeResult, SafetyConfig, finite
except ImportError:
    from core_types import RangeResult, SafetyConfig, finite


class RangeGuard:
    """Validate range policy and require stable recovery after jumps/faults."""

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.last_accepted_m: Optional[float] = None
        self.last_accepted_at: Optional[float] = None
        self.fault_reason: Optional[str] = None
        self.candidate_m: Optional[float] = None
        self.candidate_count = 0

    def _fault(self, reason: str) -> RangeResult:
        self.fault_reason = reason
        return RangeResult(False, reason)

    def _recover(self, value_m: float, now: float) -> RangeResult:
        if (
            self.candidate_m is not None
            and abs(value_m - self.candidate_m) <= self.config.jump_settle_tolerance_m
        ):
            self.candidate_count += 1
            self.candidate_m = (
                self.candidate_m * (self.candidate_count - 1) + value_m
            ) / self.candidate_count
        else:
            self.candidate_m = value_m
            self.candidate_count = 1
        if self.candidate_count < self.config.jump_recovery_samples:
            return RangeResult(False, self.fault_reason or "downward range recovering")
        self.last_accepted_m = self.candidate_m
        self.last_accepted_at = now
        self.fault_reason = None
        self.candidate_m = None
        self.candidate_count = 0
        return RangeResult(True, None)

    def observe(
        self, value_m: float, declared_min_m: float, declared_max_m: float, now: float
    ) -> RangeResult:
        if not all(finite(value) for value in (value_m, declared_min_m, declared_max_m)):
            return self._fault("downward range or declared limits are non-finite")
        if declared_min_m < 0.0 or declared_max_m <= declared_min_m:
            return self._fault("downward range declared limits are invalid")
        low = (
            self.config.configured_min_range_m
            if self.config.ignore_declared_min_range
            else max(declared_min_m, self.config.configured_min_range_m)
        )
        high = min(declared_max_m, self.config.configured_max_range_m)
        if high <= low:
            return self._fault(
                "downward range declared and configured limits do not overlap"
            )
        tolerance = self.config.range_boundary_tolerance_m
        if value_m < low - tolerance or value_m > high + tolerance:
            self.candidate_m = None
            self.candidate_count = 0
            return self._fault(
                f"downward range {value_m:.3f} m outside declared/accepted "
                f"interval [{low:.3f}, {high:.3f}] m"
            )
        if self.fault_reason is not None:
            return self._recover(value_m, now)
        if self.last_accepted_m is not None and self.last_accepted_at is not None:
            dt_s = now - self.last_accepted_at
            if dt_s < 0.0:
                return self._fault("monotonic time moved backwards")
            if (
                dt_s <= self.config.jump_window_s
                and abs(value_m - self.last_accepted_m) > self.config.max_range_jump_m
            ):
                self.candidate_m = value_m
                self.candidate_count = 1
                return self._fault(
                    f"downward range jump exceeds {self.config.max_range_jump_m:.3f} m"
                )
        self.last_accepted_m = value_m
        self.last_accepted_at = now
        return RangeResult(True, None)
