from pathlib import Path
import importlib.util

def generate_launch_description():
    spec = importlib.util.spec_from_file_location("elastic_tracker_nodes", Path(__file__).with_name("_nodes.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate("all")
