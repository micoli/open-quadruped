import dataclasses
import json
from typing import List, Optional

from pydantic import Field, TypeAdapter
from pydantic.dataclasses import dataclass
from dataclasses import dataclass
from openapi_specgen import OpenApiResponse,schema
from pydantic.dataclasses import dataclass
import sys
import inspect
import pydantic2ts

@dataclass
class DataclassObject():
    str_field: str
    int_field: int
    float_field: float
    boolean_field: bool
    list_field: List[str]


def dump_json_schema():
    for name, obj in inspect.getmembers(sys.modules[__name__]):
        print(name)
        if inspect.isclass(obj) and dataclasses.is_dataclass(obj):
            with open(f'{name}.json', 'w', encoding='utf-8') as f_out:
               f_out.write(json.dumps(TypeAdapter(obj).json_schema(),indent=4))

dump_json_schema()