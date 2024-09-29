import json
import os
import pkgutil

from pydantic import BaseModel, TypeAdapter
import importlib
import inspect


def dump_json_schema(module, path):
    for _name, package in _import_submodules(module).items():
        for name, obj in inspect.getmembers(package, inspect.isclass):
            if not (issubclass(obj, BaseModel) and obj.__base__ != BaseModel.__base__):
                continue
            export_file = os.path.realpath(os.path.join(path, f"{name}.json"))
            print(f" - {export_file}")
            with open(export_file, "w", encoding="utf-8") as f_out:
                f_out.write(json.dumps(TypeAdapter(obj).json_schema(), indent=4))


def _import_submodules(package):
    if isinstance(package, str):
        package = importlib.import_module(package)
    modules = {}
    for loader, name, is_pkg in pkgutil.walk_packages(package.__path__):
        full_name = package.__name__ + "." + name
        modules[full_name] = importlib.import_module(full_name)
        if is_pkg:
            modules.update(_import_submodules(full_name))
    return modules
