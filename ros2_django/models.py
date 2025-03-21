import logging
import importlib
from typing import Type, TypeVar, Callable

from django.core.exceptions import ObjectDoesNotExist
from django.conf import settings

from django.db import models, transaction

from .services import RosGetSrv, RosSetSrv, RosDeleteSrv, RosListSrv
from .fields import RosFieldMixin, RosManyToOneRel, RosForeignKey, RosOneToOneField
from .types import Ros2SrvDef, Ros2MsgFieldDef

logger = logging.getLogger()


PropReturn = TypeVar("PropReturn")


def classproperty(meth: Callable[..., PropReturn]) -> PropReturn:
    """Access a @classmethod like a @property."""
    # mypy doesn't understand class properties yet: https://github.com/python/mypy/issues/2563
    return classmethod(property(meth))  # type: ignore


class RosModel(models.Model):
    """Mixin that will be added to all Django models that will also be present in ROS"""

    @classproperty
    def ros_msgtype(cls: Type):  # type:ignore
        """The type name of the related ROS message"""
        return cls.__name__

    @classproperty
    def ros_rawmsgtype(cls: Type):  # type: ignore
        """The type name of the related raw ROS message"""
        return cls.__name__ + "Raw"

    @classproperty
    def ros_data_field(cls: Type):  # type: ignore
        """The ROS field name of the data field"""
        return cls.__name__.lower()

    @classproperty
    def ros_data_field_plural(cls: Type):  # type: ignore
        """The ROS field name of the data field for list purpose"""
        return cls.__name__.lower() + "s"

    @classmethod
    def services(cls) -> list[Ros2SrvDef]:
        """Create all services definitions for this model"""

        services = getattr(cls, "_services", None)
        ros_meta = getattr(cls, "RosMeta", None)

        if services:
            return services

        cls._services = []
        for raw in [False, True]:
            if raw and not cls.has_raw():
                continue

            # Get/Set/Delete
            search_fields = ["id"]
            if ros_meta:
                search_fields += getattr(ros_meta, "ros_search", [])
            for search_field in search_fields:
                cls._services.append(RosGetSrv(cls, search_field, raw))
                cls._services.append(RosSetSrv(cls, search_field, raw))
                if not raw:
                    cls._services.append(RosDeleteSrv(cls, search_field, raw))

            # List
            filter_fields = [None]
            if hasattr(cls, "RosMeta"):
                filter_fields += getattr(ros_meta, "ros_filter", [])
            for filter_field in filter_fields:
                cls._services.append(RosListSrv(cls, filter_field, raw))

        return cls._services

    @classmethod
    def msg_fields(cls, raw, thin=False) -> list[Ros2MsgFieldDef]:
        """List of fields defined in the ros msg file"""

        fields: list[Ros2MsgFieldDef] = []
        ros_meta = getattr(cls, "RosMeta", None)

        for field in cls._meta.get_fields():
            if raw and isinstance(field, models.ManyToOneRel):
                continue
            if (
                thin
                and ros_meta
                and hasattr(ros_meta, "ros_thin_fields")
                and field.name not in getattr(ros_meta, "ros_thin_fields")
            ):
                continue
            elif (
                ros_meta
                and hasattr(ros_meta, "ros_ignore_fields")
                and field.name in getattr(ros_meta, "ros_ignore_fields")
            ):
                continue
            elif isinstance(field, RosFieldMixin):
                f: Ros2MsgFieldDef = {
                    "name": field.ros_name,
                    "type": field.ros_type,
                    "field": field,  # type: ignore (no intersect type in python :()
                }
                if (
                    hasattr(field, "default")
                    and field.default != models.fields.NOT_PROVIDED
                    and not callable(field.default)
                ):
                    f["default"] = field.default
                elif field.ros_default is not None:
                    f["default"] = field.ros_default
                fields.append(f)
            else:
                logger.debug(
                    f"Ignoring field {cls.ros_msgtype}.{field.name} {type(field)}"
                )
                continue

        return fields

    @classmethod
    def has_raw(cls):
        return cls.msg_fields(True) != cls.msg_fields(False)

    def from_ros(self, ros_msg, raw, parent=None):
        """Class method to create a model object from a ros object"""

        with transaction.atomic():
            for msg_field in self.msg_fields(raw):
                field = msg_field["field"]
                if field.name == "id" and getattr(ros_msg, "id"):
                    id = getattr(ros_msg, "id")
                    if id != -1:
                        setattr(self, field.name, id)
                elif isinstance(field, RosManyToOneRel):
                    continue
                elif isinstance(field, RosForeignKey) or isinstance(
                    field, RosOneToOneField
                ):
                    rel_id = getattr(ros_msg, field.ros_name)
                    if rel_id != -1:
                        setattr(self, field.name + "_id", rel_id)
                    elif parent is not None and isinstance(parent, field.related_model):
                        setattr(self, field.name + "_id", parent.id)
                elif hasattr(ros_msg, field.ros_name):
                    setattr(
                        self, field.name, field.ros2py(getattr(ros_msg, field.ros_name))
                    )
            self.save()
            for msg_field in self.msg_fields(raw):
                field = msg_field["field"]
                if isinstance(field, RosManyToOneRel):
                    getattr(self, field.name).all().delete()

                    to_set = []
                    for child in getattr(ros_msg, field.ros_name):
                        if field.related_model:
                            m = field.related_model()
                            if isinstance(m, RosModel):
                                to_set.append(m.from_ros(child, raw, parent=self))

                    getattr(self, field.name).set(to_set)
            if hasattr(self, "check_related"):
                getattr(self, "check_related")()
        return self

    def to_ros(self, raw: bool, thin=False, fields: list[str] = []):
        """Convert model instance to a ROS message"""

        msgs = importlib.import_module(f"{settings.ROS_INTERFACES_MODULE_NAME}.msg")
        ros_msg = getattr(msgs, self.ros_msgtype if not raw else self.ros_rawmsgtype)()

        _fields = {
            f["name"]: f["field"] for f in self.msg_fields(raw, thin) if "field" in f
        }

        if fields:
            _fields = {f: _fields[f] for f in fields}

        for field in _fields.values():
            if isinstance(field, RosFieldMixin):
                if isinstance(field, models.ForeignKey):
                    if getattr(self, field.name) is None:
                        value = -1
                    else:
                        value = getattr(self, field.name).id
                else:
                    try:
                        if isinstance(field, RosManyToOneRel):
                            value = field.py2ros(getattr(self, field.name), thin=thin)
                        else:
                            value = field.py2ros(getattr(self, field.name))  # type: ignore
                    except ObjectDoesNotExist:
                        value = None
                        if field.ros_type:
                            value = getattr(msgs, field.ros_type)()
                if value is None:
                    continue
                setattr(ros_msg, field.ros_name, value)
            else:
                logger.warning(f"Ignoring field {field}")
                continue
        return ros_msg

    class Meta:
        abstract = True
