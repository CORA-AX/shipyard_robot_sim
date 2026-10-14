"""Tracking reward and command tensor operations, testable without Isaac Sim."""
import torch


FOOT_NAMES = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')


def trot_timing_score(air_time, contact_time, timing_std=0.1,
                      min_swing_time=0.06, max_swing_time=0.4):
    """Bounded diagonal timing score; columns MUST be FL, FR, RL, RR.

    Match timers within each diagonal and opposite timers across diagonals.
    Brief four-foot support is allowed (zero bonus, no penalty). All-flight,
    all-stance and reset states earn nothing. Very short swings ramp in, and
    prolonged flight decays so holding one diagonal aloft is not a solution.
    This is a gait preference, not a fixed-frequency foot trajectory.
    """
    if air_time.shape != contact_time.shape or air_time.ndim != 2 or air_time.shape[1] != 4:
        raise ValueError('Trot timing requires matching (num_envs, 4) foot timers.')
    if timing_std <= 0 or not 0 < min_swing_time < max_swing_time:
        raise ValueError('Require positive timing_std and 0 < min_swing_time < max_swing_time.')
    sync = torch.stack([
        (air_time[:, a] - air_time[:, b]).square()
        + (contact_time[:, a] - contact_time[:, b]).square()
        for a, b in ((0, 3), (1, 2))
    ], dim=1).mean(dim=1)
    alternate = torch.stack([
        (air_time[:, a] - contact_time[:, b]).square()
        + (contact_time[:, a] - air_time[:, b]).square()
        for a, b in ((0, 1), (0, 2), (3, 1), (3, 2))
    ], dim=1).mean(dim=1)
    mixed_support = (air_time > 0).any(dim=1) & (contact_time > 0).any(dim=1)
    longest_swing = air_time.max(dim=1).values
    swing_scale = (longest_swing / min_swing_time).clamp(max=1.0)
    swing_scale *= torch.exp(-(longest_swing - max_swing_time).clamp(min=0.0) / timing_std)
    return torch.exp(-(sync + alternate) / timing_std**2) * swing_scale * mixed_support


def trot_gait(env, command_name, sensor_cfg, timing_std=0.1,
              min_swing_time=0.06, max_swing_time=0.4,
              linear_threshold=0.1, angular_threshold=0.1,
              ramp_steps=0, initial_scale=1.0):
    """Moving-command gait bonus, fully enabled by default for fresh training.

    An optional run-local ramp remains available for fine-tuning. With 24
    rollout steps, 12000 policy steps is 500 PPO iterations. Every distributed
    rank advances common_step_counter equally; resume restarts this ramp.
    Sensor body_names must use FOOT_NAMES with preserve_order=True.
    """
    if ramp_steps < 0 or not 0 <= initial_scale <= 1:
        raise ValueError('Require nonnegative ramp_steps and initial_scale in [0, 1].')
    sensor = env.scene.sensors[sensor_cfg.name]
    if not sensor.cfg.track_air_time:
        raise ValueError('Trot reward requires contact sensor track_air_time=True.')
    score = trot_timing_score(
        sensor.data.current_air_time[:, sensor_cfg.body_ids],
        sensor.data.current_contact_time[:, sensor_cfg.body_ids],
        timing_std, min_swing_time, max_swing_time)
    command = env.command_manager.get_command(command_name)
    moving = (torch.linalg.vector_norm(command[:, :2], dim=1) > linear_threshold) | (
        command[:, 2].abs() > angular_threshold)
    progress = min(max(env.common_step_counter / ramp_steps, 0.0), 1.0) if ramp_steps else 1.0
    return score * moving * (initial_scale + (1.0 - initial_scale) * progress)


def feet_air_time_excess(env, command_name, sensor_cfg, max_air_time=0.4,
                         saturation_time=0.2, linear_threshold=0.1, angular_threshold=0.1):
    """Penalize each foot held airborne too long, using a negative weight.

    Normal swings up to max_air_time incur no cost. Each foot's cost increases
    linearly over saturation_time, then stays at one until touchdown. Sum over
    all four feet: a single persistently lifted foot cannot hide in an average.
    No touchdown event is required, so a foot that never lands is still charged.
    Only moving commands enable this term; it never rewards planted feet.
    """
    if max_air_time <= 0 or saturation_time <= 0:
        raise ValueError('Require positive max_air_time and saturation_time.')
    sensor = env.scene.sensors[sensor_cfg.name]
    if not sensor.cfg.track_air_time:
        raise ValueError('Air-time excess penalty requires track_air_time=True.')
    air_time = sensor.data.current_air_time[:, sensor_cfg.body_ids]
    if air_time.ndim != 2 or air_time.shape[1] != 4:
        raise ValueError('Air-time excess requires exactly four selected feet.')
    excess = ((air_time - max_air_time) / saturation_time).clamp(min=0.0, max=1.0)
    command = env.command_manager.get_command(command_name)
    moving = (torch.linalg.vector_norm(command[:, :2], dim=1) > linear_threshold) | (
        command[:, 2].abs() > angular_threshold)
    return excess.sum(dim=1) * moving


def feet_slide(env, sensor_cfg, asset_cfg, force_threshold=1.0):
    """Sum planar world-frame foot speeds only while that foot is in contact.

    The asset and sensor selections must contain the same ordered foot names.
    A short force history retains touchdown contacts between policy steps.
    Applies at standstill too, to discourage drifting while planted.
    """
    sensor = env.scene.sensors[sensor_cfg.name]
    forces = sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :]
    contact = forces.norm(dim=-1).max(dim=1).values > force_threshold
    velocity = env.scene[asset_cfg.name].data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    return (velocity.norm(dim=-1) * contact).sum(dim=1)


def sample_command_modes(commands, standing_fraction, pure_yaw_fraction, min_abs_yaw,
                         forward_fraction=0.0, forward_turn_fraction=0.0,
                         forward_speed_range=(0.2, 0.6)):
    """Mix stand, pure yaw, low-speed forward, forward turn and general commands.

    The caller supplies uniform commands with yaw bounds covering ±min_abs_yaw.
    Fractions are probabilities at each command resampling, not fixed env IDs.
    """
    fractions = (standing_fraction, pure_yaw_fraction, forward_fraction, forward_turn_fraction)
    if not (all(0 <= fraction <= 1 for fraction in fractions) and sum(fractions) <= 1):
        raise ValueError('Command mode fractions must be nonnegative and sum to at most one.')
    if min_abs_yaw <= 0:
        raise ValueError('min_abs_yaw must be positive.')
    if not 0 < forward_speed_range[0] <= forward_speed_range[1]:
        raise ValueError('Forward speeds must be positive and ordered.')
    modes = torch.rand(commands.shape[0], device=commands.device)
    standing = modes < standing_fraction
    yaw_end = standing_fraction + pure_yaw_fraction
    forward_end = yaw_end + forward_fraction
    turn_end = forward_end + forward_turn_fraction
    turning = (modes >= standing_fraction) & (modes < yaw_end)
    forward = (modes >= yaw_end) & (modes < forward_end)
    forward_turn = (modes >= forward_end) & (modes < turn_end)
    result = commands.clone()
    result[standing] = 0.0
    result[turning, :2] = 0.0
    advancing = forward | forward_turn
    result[advancing, 0] = torch.empty_like(result[advancing, 0]).uniform_(*forward_speed_range)
    result[advancing, 1] = 0.0
    result[forward, 2] = 0.0
    rotating = turning | forward_turn
    yaw = result[rotating, 2]
    result[rotating, 2] = torch.where(yaw >= 0, yaw.clamp(min=min_abs_yaw), yaw.clamp(max=-min_abs_yaw))
    return result, standing, turning


def track_lin_vel_xy_error(env, command_name, asset_cfg):
    """Planar body-frame speed error (m/s), used with a negative weight."""
    command = env.command_manager.get_command(command_name)
    velocity = env.scene[asset_cfg.name].data.root_lin_vel_b
    return torch.linalg.vector_norm(command[:, :2] - velocity[:, :2], dim=1)


def track_ang_vel_z_error(env, command_name, asset_cfg, pure_yaw_scale=3.0,
                        linear_threshold=0.1, angular_threshold=0.1):
    """Absolute yaw error; emphasize pure turns while also penalizing yaw drift.

    The mode is derived from the actual command, so fixed-command evaluation
    and command changes do not depend on sampler-specific environment IDs.
    """
    command = env.command_manager.get_command(command_name)
    yaw = env.scene[asset_cfg.name].data.root_ang_vel_b[:, 2]
    pure_yaw = (torch.linalg.vector_norm(command[:, :2], dim=1) <= linear_threshold) & (
        command[:, 2].abs() > angular_threshold)
    scale = torch.where(pure_yaw, pure_yaw_scale, 1.0)
    return (command[:, 2] - yaw).abs() * scale


def feet_air_time_tracking(env, command_name, sensor_cfg, min_air_time=0.1, max_air_time=0.4,
                           linear_threshold=0.1, angular_threshold=0.1):
    """Reward bounded swing duration at touchdown for translation OR rotation.

    Standing and continuous contact yield zero. Tiny contact chatter gets no
    bonus, and long airborne periods cannot increase the bonus without bound.
    The command thresholds have separate units: m/s and rad/s.
    """
    sensor = env.scene.sensors[sensor_cfg.name]
    first_contact = sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    air_time = sensor.data.last_air_time[:, sensor_cfg.body_ids]
    touchdown_bonus = (air_time - min_air_time).clamp(min=0.0, max=max_air_time - min_air_time)
    command = env.command_manager.get_command(command_name)
    moving = (torch.linalg.vector_norm(command[:, :2], dim=1) > linear_threshold) | (
        command[:, 2].abs() > angular_threshold)
    return (touchdown_bonus * first_contact).sum(dim=1) * moving
