"""Resolve assets and launch an isolated Isaac worker from system Humble."""
from pathlib import Path

from robot_sim.config import load_world_config
from robot_sim.environment import isaac_environment


def default_project_root():
    for start in (Path(__file__).resolve(), Path.cwd()):
        for path in (start, *start.parents):
            if (path / 'assets/go2/config/sources.lock.json').is_file():
                return str(path)
    return ''


def launch_simulator(context):
    from launch.actions import ExecuteProcess
    from launch.substitutions import LaunchConfiguration
    def value(name):
        return LaunchConfiguration(name).perform(context)
    root = Path(value('project_root')).resolve()
    if not (root / 'assets/go2/config/sources.lock.json').is_file():
        raise ValueError('Set project_root to the simulation repository containing assets/.')
    config_path = Path(value('robots_file')).resolve()
    load_world_config(config_path)  # Fail before starting the GPU for unsupported robots/configs.
    python = value('isaac_python') or str(root / '.conda/isaacsim-5.0/bin/python')
    if not Path(python).is_file():
        raise ValueError(f'Isaac Python does not exist: {python}')
    domain = int(value('ros_domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('ROS domain must be 0..232.')
    command = [python, '-m', 'robot_sim.bootstrap', '--project-root', str(root),
               '--robots-file', str(config_path), '--device', value('device'),
               '--policy-device', value('policy_device'), '--steps', value('steps'),
               '--command-timeout', value('command_timeout'), '--ros-domain-id', str(domain)]
    if value('report'):
        command += ['--report', str(Path(value('report')).resolve())]
    if value('checkpoint'):
        command += ['--checkpoint', str(Path(value('checkpoint')).resolve())]
    mode = value('display')
    if mode == 'headless':
        command += ['--headless']
    elif mode == 'webrtc':
        if not value('server_ip'):
            raise ValueError('Set server_ip for WebRTC.')
        command += ['--webrtc', '--server-ip', value('server_ip')]
    elif mode != 'local':
        raise ValueError('display must be local, headless or webrtc.')
    paths = [root, root / 'ros2_ws/src/robot_sim', root / 'ros2_ws/src/robot_control']
    environment = isaac_environment(python, python_paths=paths)
    environment['ROS_DOMAIN_ID'] = str(domain)
    if mode == 'local':
        environment.update(HEADLESS='0', LIVESTREAM='0')
    # Validate the viewer before starting the expensive simulator process.
    from .rviz_launch import launch_rviz
    viewers = launch_rviz(context, simulation_display=mode)
    return [ExecuteProcess(cmd=command, cwd=str(root), env=environment,
                           output='screen', sigterm_timeout='15', sigkill_timeout='15'), *viewers]
