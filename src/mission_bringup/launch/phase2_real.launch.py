import importlib.util
from pathlib import Path


def generate_launch_description():
    target = Path(__file__).with_name('integrated.launch.py')
    spec = importlib.util.spec_from_file_location('mission_bringup_integrated_launch', target)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Unable to load launch file: {target}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
