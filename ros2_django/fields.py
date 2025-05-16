import datetime
import json
from typing import Type

from django.db import models
from django.core.exceptions import ValidationError
import jsonschema
from jsonschema.exceptions import ValidationError as JsonValidationError
from rclpy.serialization import serialize_message, deserialize_message  # type:ignore

from .utils import ros2json, json2ros

#
# Ros fields
#


class RosFieldMixin(object):
    """Mixin for all fields inside model with RosModelMixin"""

    ros_type: str = "error"
    ros_default = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def ros_name(self) -> str:
        return self.name  # type: ignore

    def py2ros(self, value):
        """Method for converting python object to its ros counterpart"""
        return value  # for basic types, ros and python are equivalent

    def ros2py(self, value):
        """Method for converting ros object to its python counterpart"""
        return value  # for basic types, ros and python are equivalent


class RosCharField(RosFieldMixin, models.CharField):
    ros_type = "string"


class RosTextField(RosFieldMixin, models.TextField):
    ros_type = "string"


class RosBooleanField(RosFieldMixin, models.BooleanField):
    ros_type = "bool"


class RosIntegerField(RosFieldMixin, models.IntegerField):
    ros_type = "int32"


class RosBigAutoField(RosFieldMixin, models.BigAutoField):
    ros_type = "int64"


class RosFloatField(RosFieldMixin, models.FloatField):
    ros_type = "float32"


class RosUUIDField(RosFieldMixin, models.UUIDField):
    ros_type = "string"

    def py2ros(self, value):
        return str(value)


class RosDateTimeField(RosFieldMixin, models.DateTimeField):
    ros_type = "int64"

    def py2ros(self, value):
        return int(value.timestamp())

    def ros2py(self, value):
        return datetime.datetime.fromtimestamp(value)


class RosBinaryField(RosFieldMixin, models.BinaryField):
    ros_type = "uint8[]"

    def __init__(self, *args, **kwargs):
        editable = kwargs.pop("editable", True)
        super().__init__(*args, **kwargs, editable=editable)

    def py2ros(self, value):
        if value is None:
            return []
        return value

    def ros2py(self, value):
        if value == []:
            return None

        return bytes(value)


class RosImageField(RosBinaryField):
    pass


class RosJSONField(RosFieldMixin, models.JSONField):
    ros_type = "string"
    ros_default = '"{}"'

    def __init__(self, *args, **kwargs):
        self.json_schema = kwargs.pop(
            "json_schema", getattr(self.__class__, "json_schema", None)
        )
        super().__init__(*args, **kwargs)

    def validate(self, value, model_instance):
        if self.json_schema:
            try:
                jsonschema.validate(value, self.json_schema)
            except JsonValidationError as e:
                raise ValidationError(f'Value "{value}" failed schema validation : {e}')
        return super().validate(value, model_instance)

    def py2ros(self, value):
        return json.dumps(value)

    def ros2py(self, value):
        return json.loads(value)


class RosMsgField(RosFieldMixin, models.Field):
    description = "ROS msg data stored as binary (optimized but type should not change)"
    empty_values = [None]

    def __init__(self, *args, ros_msg: Type, ros_type: str, **kwargs):
        super().__init__(*args, **kwargs)
        self.ros_msg = ros_msg
        self.ros_type = ros_type

    def deconstruct(self):
        name, path, args, kwargs = super().deconstruct()
        kwargs["ros_msg"] = self.ros_msg
        kwargs["ros_type"] = self.ros_type
        return name, path, args, kwargs

    #
    # Internal type is binary
    #

    def get_internal_type(self):
        return "BinaryField"

    def get_db_prep_value(self, value, connection, prepared=False):
        value = super().get_db_prep_value(value, connection, prepared)
        if value is not None:
            return connection.Database.Binary(value)
        return value

    # ROS/Python conversion, value is from db (bytes)

    def py2ros(self, value):
        if value is None:
            return self.ros_msg()
        return deserialize_message(value, self.ros_msg)

    def ros2py(self, value):
        return serialize_message(value)

    # DB/Fields conversion (for admin interface)

    def value_from_object(self, obj):
        value = super().value_from_object(obj)
        return ros2json(self.py2ros(value))

    def to_python(self, value):
        return self.ros2py(json2ros(value, self.ros_msg))

    # DB/string conversion (for loaddata/dumpdata)
    def value_to_string(self, obj):
        return self.value_from_object(obj)


class RosMsgFieldJSON(RosFieldMixin, models.JSONField):
    description = "ROS msg data stored as JSON (safer)"
    empty_values = [None]

    def __init__(self, *args, ros_msg: Type, ros_type: str, **kwargs):
        super().__init__(*args, **kwargs)
        self.ros_msg = ros_msg
        self.ros_type = ros_type

    def deconstruct(self):
        name, path, args, kwargs = super().deconstruct()
        kwargs["ros_msg"] = self.ros_msg
        kwargs["ros_type"] = self.ros_type
        return name, path, args, kwargs

    # ROS/Python conversion
    def py2ros(self, value):
        if value is None:
            return self.ros_msg()
        return json2ros(json.dumps(value), self.ros_msg)

    def ros2py(self, value):
        return json.loads(ros2json(value))


class RosManyToOneRel(RosFieldMixin, models.ManyToOneRel):
    @property
    def ros_type(self):
        from .models import RosModel

        if issubclass(self.related_model, RosModel):
            return self.related_model.ros_msgtype + "[]"

        raise Exception("ROS related field should be related to a RosModel")

    def py2ros(self, values, thin=False):
        return [v.to_ros(False, thin) for v in values.all()]


class RosForeignKey(RosFieldMixin, models.ForeignKey):
    ros_type = "int64"
    ros_default = -1
    rel_class = RosManyToOneRel

    def __init__(self, *args, **kwargs):
        kwargs["related_name"] = "%(class)ss"
        super().__init__(*args, **kwargs)

    @property
    def ros_name(self):
        return "id_" + self.name


class RosOneToOneRel(RosFieldMixin, models.OneToOneRel):
    @property
    def ros_type(self):
        from .models import RosModel

        if issubclass(self.related_model, RosModel):
            return self.related_model.ros_msgtype

        raise Exception("ROS related field should be related to a RosModel")

    def py2ros(self, value):
        return value.to_ros(False)


class RosOneToOneField(RosFieldMixin, models.OneToOneField):
    ros_type = "int64"
    ros_default = -1
    rel_class = RosOneToOneRel

    @property
    def ros_name(self):
        return "id_" + self.name
