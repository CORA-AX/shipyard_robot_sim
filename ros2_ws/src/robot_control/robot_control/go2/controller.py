"""Go2 observation/action adapter; the simulator owns stepping and time."""
from .policy import JOINT_NAMES


class Go2Controller:
    def __init__(self, env, policy):
        import torch
        self.env, self.policy = env, policy
        self.robot = env.scene['robot']
        self.command_term = env.command_manager.get_term('base_velocity')
        self.command_tensor = torch.zeros((1, 3), dtype=torch.float32, device=env.device)
        observation = env.observation_manager.compute()['policy']
        if (self.robot.joint_names != JOINT_NAMES or observation.shape != (1, 48)
                or env.action_manager.total_action_dim != 12 or env.step_dt != .02):
            raise RuntimeError('Go2 requires its 48x12 observation/action contract at 50Hz.')

    def observation(self, command):
        import torch
        self.command_tensor[0] = torch.tensor(command, device=self.env.device)
        self.command_term.vel_command_b[:] = self.command_tensor
        observation = self.env.observation_manager.compute()['policy']
        if not torch.equal(observation[:, 9:12], self.command_tensor):
            raise RuntimeError('Command differs from Go2 policy observation.')
        return observation

    def actions(self, observation):
        import torch
        return torch.tensor([self.policy.actions(observation[0].tolist())], device=self.env.device)


class ArticulationGo2Controller:
    """The same flat-policy contract without an environment or sim.step()."""
    def __init__(self, robot, policy):
        import torch
        if robot.joint_names != JOINT_NAMES or robot.data.joint_pos.shape != (1, 12):
            raise RuntimeError('Go2 requires the registered 12-joint order.')
        self.robot, self.policy = robot, policy
        self.last_action = torch.zeros_like(robot.data.joint_pos)

    def reset(self):
        self.last_action.zero_()
        self.robot.set_joint_position_target(self.robot.data.default_joint_pos)

    def observation(self, command):
        import torch
        data = self.robot.data
        velocity = torch.tensor([command], device=self.last_action.device, dtype=self.last_action.dtype)
        return torch.cat((data.root_lin_vel_b, data.root_ang_vel_b, data.projected_gravity_b,
                          velocity, data.joint_pos - data.default_joint_pos,
                          data.joint_vel - data.default_joint_vel, self.last_action), dim=-1)

    def apply(self, command):
        import torch
        observation = self.observation(command)
        action = torch.tensor([self.policy.actions(observation[0].tolist())],
                              device=self.last_action.device, dtype=self.last_action.dtype)
        self.robot.set_joint_position_target(self.robot.data.default_joint_pos + .25 * action)
        self.last_action.copy_(action)
        return observation
