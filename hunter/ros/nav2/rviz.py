"""Prepare only RViz's graphics environment; leave simulation/Nav2 untouched."""
import os


def rviz_environment(environment, rendering='software'):
    if rendering not in ('software','native'):
        raise ValueError('RViz rendering must be software or native.')
    result = dict(environment)
    for key in ('PYTHONHOME','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH','QML2_IMPORT_PATH'):
        result.pop(key,None)
    for key in ('PATH','PYTHONPATH','LD_LIBRARY_PATH'):
        if key in result:
            result[key] = os.pathsep.join(path for path in result[key].split(os.pathsep)
                if path and not any(part in path.lower() for part in
                    ('/conda','/.conda','/miniconda','/anaconda','/isaacsim','/omni.')))
    if rendering == 'software':
        result.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa')
    return result
