#!/usr/bin/env python3
"""MHE -> NMPC continuous payload-estimate interface.

The ROS transport is ``std_msgs/Float64MultiArray`` so this package does not
need a custom-message build step.  Keeping the schema and the payload algebra
here prevents the estimator and controller from silently disagreeing about
array indices or the meaning of ``s_xy``.
"""

from dataclasses import dataclass

import numpy as np


FIELDS = (
    'm_total', 's_x', 's_y', 'c_x', 'c_y',
    'dJ_xx', 'dJ_yy', 'dJ_zz', 'J_xx', 'J_yy', 'J_zz',
    'no_payload_confidence', 'healthy', 'solution_age_sec',
)
SIZE = len(FIELDS)
LEGACY_SIZE = 12


def _smooth_zero_score(value, full, zero):
    """Return 1 below ``full`` and 0 above ``zero`` with a C1 transition."""
    value = abs(float(value))
    if value <= full:
        return 1.0
    if value >= zero:
        return 0.0
    x = (value - full) / max(zero - full, 1e-12)
    return 1.0 - (3.0 * x * x - 2.0 * x * x * x)


def no_payload_confidence(m_payload, s_xy, dJ_diag,
                          mass_full=0.015, mass_zero=0.060,
                          moment_full=0.0015, moment_zero=0.0060,
                          inertia_full=0.0020, inertia_zero=0.0100):
    """Fuse three independent "empty" indications into a [0, 1] score.

    A product is intentional: one clear payload indication is sufficient to
    veto "empty".  In particular, a non-zero first mass moment keeps the score
    low even when the scalar mass estimate happens to touch its lower bound.
    """
    s_norm = float(np.linalg.norm(np.asarray(s_xy, dtype=float)))
    dj_xy = float(np.max(np.abs(np.asarray(dJ_diag, dtype=float)[:2])))
    return float(np.clip(
        _smooth_zero_score(m_payload, mass_full, mass_zero)
        * _smooth_zero_score(s_norm, moment_full, moment_zero)
        * _smooth_zero_score(dj_xy, inertia_full, inertia_zero),
        0.0, 1.0))


def inertia_from_mass_moment(m_total, s_xy, m_body, J_body, rz,
                             payload_ki=0.0, dJ_xy=None):
    """Return ``(c_xy, dJ_diag, J_diag)`` from ``m`` and ``s=m_P*r_xy``.

    This is the diagonal part of the same first-moment model used by
    :mod:`mhe_model`.  The ill-conditioned O(r_xy**2) terms are deliberately
    omitted there and here.  The off-diagonal terms remain internal to the MHE;
    the current NMPC interface consumes the roll/pitch diagonal increments.
    """
    m_total = max(float(m_total), 1e-6)
    m_payload = max(m_total - float(m_body), 0.0)
    s_xy = np.asarray(s_xy, dtype=float).reshape(2)
    J_body = np.asarray(J_body, dtype=float).reshape(3)
    c_xy = s_xy / m_total
    intrinsic = float(payload_ki) * m_payload
    if dJ_xy is None:
        mu = float(m_body) * m_payload / m_total
        dJ_xy = mu * float(rz) ** 2 + intrinsic
    dJ_diag = np.array([max(float(dJ_xy), 0.0),
                        max(float(dJ_xy), 0.0), intrinsic])
    return c_xy, dJ_diag, J_body + dJ_diag


@dataclass(frozen=True)
class PayloadEstimate:
    m_total: float
    s_xy: np.ndarray
    c_xy: np.ndarray
    dJ_diag: np.ndarray
    J_diag: np.ndarray
    no_payload_confidence: float
    # ``healthy`` is the health of the most recent MHE solve *and* its input
    # streams.  ``solution_age_sec`` is independent of ROS transport age: MHE
    # may keep publishing a held last-good estimate after a failed solve.
    healthy: bool = False
    solution_age_sec: float = 1.0e6

    def as_array(self):
        return np.array([
            self.m_total, *self.s_xy, *self.c_xy, *self.dJ_diag,
            *self.J_diag, self.no_payload_confidence,
            1.0 if self.healthy else 0.0, self.solution_age_sec,
        ], dtype=float)

    @classmethod
    def from_array(cls, values):
        a = np.asarray(values, dtype=float)
        # A rolling deployment may briefly expose a new NMPC to the old
        # 12-field publisher.  Decode it as explicitly unhealthy rather than
        # crashing or, worse, treating an unqualified estimate as fresh.
        if a.shape not in ((SIZE,), (LEGACY_SIZE,)) or not np.all(np.isfinite(a)):
            raise ValueError(
                f'payload estimate must be {SIZE} finite values '
                f'({LEGACY_SIZE}-field legacy frames are accepted unhealthy)')
        healthy = bool(a[12] >= 0.5) if a.shape == (SIZE,) else False
        solution_age = max(float(a[13]), 0.0) if a.shape == (SIZE,) else 1.0e6
        return cls(
            m_total=float(a[0]), s_xy=a[1:3].copy(), c_xy=a[3:5].copy(),
            dJ_diag=a[5:8].copy(), J_diag=a[8:11].copy(),
            no_payload_confidence=float(np.clip(a[11], 0.0, 1.0)),
            healthy=healthy, solution_age_sec=solution_age)


def slew(value, target, max_rate, dt):
    """Scalar/vector slew limiter used by the NMPC model and gain schedule."""
    value = np.asarray(value, dtype=float)
    target = np.asarray(target, dtype=float)
    step = max(float(max_rate), 0.0) * max(float(dt), 0.0)
    out = value + np.clip(target - value, -step, step)
    return float(out) if out.ndim == 0 else out


def headroom_limited_scale(current, command, requested_scale, limit):
    """Limit a common roll/pitch error scale before body-rate saturation.

    The returned scale stays at least one, so scheduling never makes the raw
    NMPC command less authoritative.  A common scale preserves the direction
    of the roll/pitch rate-error vector.
    """
    current = np.asarray(current, dtype=float).reshape(-1)
    command = np.asarray(command, dtype=float).reshape(-1)
    if current.shape != command.shape or not np.all(np.isfinite(current)) \
            or not np.all(np.isfinite(command)):
        raise ValueError('current and command must be equally shaped and finite')
    limit = float(limit)
    if not np.isfinite(limit) or limit <= 0.0:
        raise ValueError('limit must be positive and finite')
    scale = max(1.0, float(requested_scale))
    for w_now, w_cmd in zip(current, command):
        error = w_cmd - w_now
        if abs(error) <= 1e-12:
            continue
        bound = ((limit - w_now) / error if error > 0.0
                 else (-limit - w_now) / error)
        scale = min(scale, max(1.0, float(bound)))
    return float(scale)


def relative_error_percent(value, reference, eps=1e-6):
    """Finite-safe diagnostic relative error; undefined references yield NaN."""
    reference = float(reference)
    if not np.isfinite(reference) or abs(reference) <= abs(float(eps)):
        return float('nan')
    return float((float(value) - reference) / reference * 100.0)
