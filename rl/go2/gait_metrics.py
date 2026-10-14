"""Offline gait measurements for arrays ordered FL, FR, RL, RR."""
import numpy as np

FOOT_NAMES = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')


def _stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    return {'count': int(values.size), 'mean': float(values.mean()),
            'median': float(np.median(values)), 'p95': float(np.percentile(values, 95))}


def summarize_gait(contact, foot_height, foot_speed, dt):
    """Summarize (steps, envs, 4) arrays with NaNs at invalid/reset samples.

    Heights are foot-body origins above the flat ground. Swing peak lift is
    measured relative to each foot's median stance height, not its USD origin.
    Partial swings and cycles spanning a reset/warm-up gap are excluded.
    Contact agreement alone can be high at standstill: use alternating support,
    complete cycles and per-foot duty factors together when judging trot.
    """
    contact = np.asarray(contact, dtype=float)
    foot_height = np.asarray(foot_height, dtype=float)
    foot_speed = np.asarray(foot_speed, dtype=float)
    if contact.ndim != 3 or contact.shape[-1] != 4:
        raise ValueError('Expected contact arrays with shape (steps, envs, 4).')
    if foot_height.shape != contact.shape or foot_speed.shape != contact.shape or dt <= 0:
        raise ValueError('Foot arrays must have identical shapes and dt must be positive.')
    valid = np.isfinite(contact).all(axis=-1)
    c = contact > 0.5
    alternating = ((c[..., 0] & c[..., 3] & ~c[..., 1] & ~c[..., 2])
                   | (c[..., 1] & c[..., 2] & ~c[..., 0] & ~c[..., 3]))
    result = {'foot_order': list(FOOT_NAMES), 'valid_frames': int(valid.sum()),
              'alternating_diagonal_support_fraction': float(alternating[valid].mean()) if valid.any() else None,
              'all_stance_fraction': float(c.all(axis=-1)[valid].mean()) if valid.any() else None,
              'all_flight_fraction': float((~c.any(axis=-1))[valid].mean()) if valid.any() else None,
              'diagonal_contact_agreement': {}, 'feet': {}, 'diagonal_event_delay_s': {}}
    for a, b in ((0, 3), (1, 2)):
        label = f'{FOOT_NAMES[a]}+{FOOT_NAMES[b]}'
        result['diagonal_contact_agreement'][label] = (
            float((c[..., a] == c[..., b])[valid].mean()) if valid.any() else None)
        # Matched events must be in one uninterrupted valid segment and within
        # 0.25 s. Report unmatched events as well so missing steps cannot hide.
        for landing in (True, False):
            delays, unmatched = [], 0
            for env_id in range(contact.shape[1]):
                events = []
                for foot in (a, b):
                    transition = (c[1:, env_id, foot] != c[:-1, env_id, foot])
                    transition &= c[1:, env_id, foot] == landing
                    transition &= valid[1:, env_id] & valid[:-1, env_id]
                    events.append(np.flatnonzero(transition) + 1)
                available = list(events[1])
                for index in events[0]:
                    candidates = [other for other in available if abs(other - index) * dt <= 0.25
                                  and valid[min(index, other):max(index, other) + 1, env_id].all()]
                    if not candidates:
                        unmatched += 1
                        continue
                    other = min(candidates, key=lambda value: abs(value - index))
                    available.remove(other)
                    delays.append(abs(other - index) * dt)
                unmatched += len(available)
            result['diagonal_event_delay_s'][f'{label}:{"touchdown" if landing else "liftoff"}'] = {
                'matched_delay': _stats(delays), 'unmatched_events': unmatched}
    for foot, name in enumerate(FOOT_NAMES):
        periods, swings, peaks = [], [], []
        for env_id in range(contact.shape[1]):
            stance = valid[:, env_id] & c[:, env_id, foot]
            heights = foot_height[:, env_id, foot]
            baseline_values = heights[stance & np.isfinite(heights)]
            baseline = np.median(baseline_values) if baseline_values.size else np.nan
            adjacent = valid[1:, env_id] & valid[:-1, env_id]
            lift = np.flatnonzero(adjacent & c[:-1, env_id, foot] & ~c[1:, env_id, foot]) + 1
            land = np.flatnonzero(adjacent & ~c[:-1, env_id, foot] & c[1:, env_id, foot]) + 1
            for start, end in zip(lift[:-1], lift[1:]):
                if valid[start:end + 1, env_id].all():
                    periods.append((end - start) * dt)
            for start in lift:
                ends = land[land > start]
                if not ends.size:
                    continue
                end = ends[0]
                if valid[start:end + 1, env_id].all():
                    swings.append((end - start) * dt)
                    peaks.append(np.max(heights[start:end]) - baseline)
        foot_valid = valid & np.isfinite(foot_height[..., foot]) & np.isfinite(foot_speed[..., foot])
        result['feet'][name] = {
            'stance_fraction': float(c[..., foot][valid].mean()) if valid.any() else None,
            'stance_slip_m_s': _stats(foot_speed[..., foot][foot_valid & c[..., foot]]),
            'cycle_period_s': _stats(periods), 'swing_duration_s': _stats(swings),
            'swing_peak_lift_above_stance_m': _stats(peaks)}
    return result
