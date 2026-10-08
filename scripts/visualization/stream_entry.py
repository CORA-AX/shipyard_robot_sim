#!/usr/bin/env python3
"""Apply the Isaac Sim 5.0 display startup profile, then run a simulation script."""
from pathlib import Path
import runpy
import sys


def configure_streaming_app():
    from isaacsim import SimulationApp

    # CLI settings are reset by SimulationApp.reset_render_settings().
    # Avoid creating another empty stage before the display is ready.
    SimulationApp.DEFAULT_LAUNCHER_CONFIG.update(sync_loads=False, hide_ui=False, create_new_stage=False)
    from isaaclab.app import AppLauncher

    original_init = AppLauncher.__init__

    def launch(app_launcher, *args, **kwargs):
        original_init(app_launcher, *args, **kwargs)
        import carb

        carb.settings.get_settings().set_bool('/omni.kit.plugin/syncUsdLoads', True)

    AppLauncher.__init__ = launch


def main():
    script = Path(sys.argv[1]).resolve()
    sys.argv = [str(script), *sys.argv[2:]]
    sys.path.insert(0, str(script.parent))
    configure_streaming_app()
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
