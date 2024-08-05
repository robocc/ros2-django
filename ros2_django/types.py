from typing import Protocol, Callable, TypedDict, Any, Type, cast, TYPE_CHECKING
from typing_extensions import NotRequired

from django.db.models import Field
from .fields import RosFieldMixin


class GenericField(RosFieldMixin, Field): ...


class Ros2MsgFieldDef(TypedDict):
    type: str
    name: str
    enum: NotRequired[list[tuple[str, Any]]]
    default: NotRequired[Any]
    field: NotRequired[GenericField]


class Ros2SrvDef(Protocol):
    name: str
    inputs: list[Ros2MsgFieldDef]
    outputs: list[Ros2MsgFieldDef]
    cb: Callable
