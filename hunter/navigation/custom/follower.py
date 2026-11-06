"""Pure-pursuit single-goal mission and simulation-time metrics, without ROS."""
from dataclasses import asdict
import math
from pathlib import Path
import time

import yaml
from hunter.control.commands import DriveCommand
from hunter.navigation.custom.path import plan
from hunter.navigation.state import wrap


def load_navigation(root, robot, path=None):
    config = yaml.safe_load(Path(path or Path(root)/'config/navigation.yaml').read_text())
    required = ('cruise_speed', 'lookahead', 'turning_radius_factor', 'approach_distance',
                'position_tolerance', 'yaw_tolerance', 'stopped_speed', 'stopped_yaw_rate',
                'arrival_hold_time', 'deceleration', 'max_lateral_acceleration', 'max_tracking_error')
    if not isinstance(config, dict):
        raise ValueError('Navigation configuration must be a mapping.')
    for name in required:
        if type(config.get(name)) not in (int, float) or not math.isfinite(config[name]) or config[name] <= 0.:
            raise ValueError(f'Navigation {name} must be finite and positive.')
    if (config['turning_radius_factor'] < 1. or config['cruise_speed'] > robot['max_speed']
            or config['deceleration'] > robot['max_acceleration']):
        raise ValueError('Navigation radius/speed/deceleration must respect vehicle limits.')
    return config


class Mission:
    def __init__(self, start, goal, robot, config):
        self.robot, self.config, self.start, self.goal = robot, config, start, goal
        radius = robot['wheelbase']/math.tan(robot['max_steering_angle'])*config['turning_radius_factor']
        begin = time.monotonic()
        already_near = (math.hypot(start.pose.x-goal.x, start.pose.y-goal.y) <= config['position_tolerance']
                        and abs(wrap(start.pose.yaw-goal.yaw)) <= config['yaw_tolerance'])
        self.path = plan(start.pose, start.pose if already_near else goal, radius, config['approach_distance'])
        self.planning_wall_s = time.monotonic()-begin
        self.progress = self.distance = 0.
        self.previous = start
        self.state, self.reason = 'tracking', None
        self.arrival_since = None
        self.finished_at = None
        self.command = DriveCommand()
        self.cross_track = 0.

    @property
    def done(self):
        return self.state in ('succeeded', 'failed', 'cancelled')

    def stop(self, reason, cancelled=False):
        if not self.done:
            self.state = 'cancelled' if cancelled else 'failed'
            self.reason = reason
            self.finished_at = self.previous.seconds
            self.command = DriveCommand(steering_angle=self.command.steering_angle)
        return self.command

    def update(self, observation):
        if self.done:
            return self.command
        if observation.seconds < self.previous.seconds-1e-9:
            return self.stop('simulation_time_reset')
        if observation.seconds == self.previous.seconds and observation != self.start:
            return self.command
        self.distance += math.hypot(observation.pose.x-self.previous.pose.x, observation.pose.y-self.previous.pose.y)
        self.previous = observation
        self.progress, self.cross_track = self.path.project(observation.pose, self.progress)
        remaining = self.path.length-self.progress
        error = math.hypot(observation.pose.x-self.goal.x, observation.pose.y-self.goal.y)
        yaw_error = abs(wrap(observation.pose.yaw-self.goal.yaw))
        cfg = self.config
        near = (error <= cfg['position_tolerance'] and yaw_error <= cfg['yaw_tolerance']
                and remaining <= max(cfg['lookahead'], cfg['position_tolerance']))
        if near:
            self.state = 'settling'
            self.command = DriveCommand(steering_angle=self.command.steering_angle)
            if abs(observation.speed) <= cfg['stopped_speed'] and abs(observation.yaw_rate) <= cfg['stopped_yaw_rate']:
                if self.arrival_since is None:
                    self.arrival_since = observation.seconds
                if observation.seconds-self.arrival_since >= cfg['arrival_hold_time']-1e-9:
                    self.state, self.finished_at = 'succeeded', observation.seconds
            else:
                self.arrival_since = None
            return self.command
        self.arrival_since = None
        self.state = 'tracking'
        if self.cross_track > cfg['max_tracking_error']:
            return self.stop('tracking_error')
        if remaining < .02 and error > cfg['position_tolerance']:
            return self.stop('goal_overshoot')
        if remaining < .02 and yaw_error > cfg['yaw_tolerance']:
            return self.stop('goal_heading_missed')
        target = self.path.at(self.progress+cfg['lookahead'])
        dx, dy = target.x-observation.pose.x, target.y-observation.pose.y
        local_y = -math.sin(observation.pose.yaw)*dx + math.cos(observation.pose.yaw)*dy
        square = dx*dx + dy*dy
        curvature = 2.*local_y/max(square, 1e-9)
        steering = max(-self.robot['max_steering_angle'], min(self.robot['max_steering_angle'],
                       math.atan(self.robot['wheelbase']*curvature)))
        speed = min(cfg['cruise_speed'], math.sqrt(cfg['max_lateral_acceleration']/max(abs(curvature), 1e-9)),
                    math.sqrt(2.*cfg['deceleration']*max(0., remaining)), 1.2*remaining)
        self.command = DriveCommand(speed, steering, acceleration=cfg['deceleration'])
        return self.command

    def result(self):
        final = self.previous
        elapsed = (self.finished_at if self.finished_at is not None else final.seconds)-self.start.seconds
        return dict(status=self.state, reason=self.reason, goal=asdict(self.goal),
                    start=asdict(self.start), final=asdict(final),
                    elapsed_sim_s=max(0., elapsed), time_cost_s=elapsed if self.state == 'succeeded' else None,
                    planning_wall_s=self.planning_wall_s, travelled_distance_m=self.distance,
                    planned_distance_m=self.path.length, progress_m=self.progress,
                    position_error_m=math.hypot(final.pose.x-self.goal.x, final.pose.y-self.goal.y),
                    yaw_error_rad=abs(wrap(final.pose.yaw-self.goal.yaw)),
                    configuration=dict(self.config), reference_point='rear_axle_midpoint',
                    mode='forward_only_no_obstacle_avoidance')
