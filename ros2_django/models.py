import logging
import importlib
from typing import Type

from django.core.exceptions import ObjectDoesNotExist
from django.conf import settings

from django.db import models, transaction

from .services import RosGetSrv, RosSetSrv, RosDeleteSrv, RosListSrv, RosSrv
from .fields import RosFieldMixin, RosManyToOneRel, RosForeignKey, RosOneToOneField

logger = logging.getLogger()


class classproperty(property):
    def __get__(self, owner_self, owner_cls):
        if self.fget:
            return self.fget(owner_cls)


class RosModel(models.Model):
    """Mixin that will be added to all Django models that will also be present in ROS"""

    @classproperty
    def ros_msgtype(cls: Type):
        """The type name of the related ROS message"""
        return cls.__name__

    @classproperty
    def ros_rawmsgtype(cls: Type):
        """The type name of the related raw ROS message"""
        return cls.__name__ + "Raw"

    @classproperty
    def ros_data_field(cls: Type):
        """The ROS field name of the data field"""
        return cls.__name__.lower()

    @classproperty
    def ros_data_field_plural(cls: Type):
        """The ROS field name of the data field for list purpose"""
        return cls.__name__.lower() + "s"

    @classmethod
    def services(cls) -> list[RosSrv]:
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
    def msg_fields(cls, raw, thin=False):
        """List of fields defined in the ros msg file"""

        fields = []
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
                f = {"name": field.ros_name, "type": field.ros_type, "field": field}
                if field.ros_default is not None:
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
                    for child in getattr(ros_msg, field.ros_name):  # type: ignore
                        m = field.related_model()
                        if isinstance(m, RosModel):
                            to_set.append(m.from_ros(child, raw, parent=self))

                    getattr(self, field.name).set(to_set)
        return self

    def to_ros(self, raw, thin=False):
        """Convert model instance to a ROS message
        (need `data_manager` import from rosidl_generate_interfaces)"""
        msgs = importlib.import_module(f"{settings.ROS_INTERFACES_MODULE_NAME}.msg")

        ros_msg = getattr(msgs, self.ros_msgtype if not raw else self.ros_rawmsgtype)()

        for _field in self.msg_fields(raw, thin):
            field = _field["field"]
            if isinstance(field, RosFieldMixin) and isinstance(field, models.Field):
                if isinstance(field, models.ForeignKey):
                    value = getattr(self, field.name).id
                else:
                    try:
                        if isinstance(field, RosManyToOneRel):
                            value = field.py2ros(getattr(self, field.name), thin=thin)
                        else:
                            value = field.py2ros(getattr(self, field.name))
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
