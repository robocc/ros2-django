import logging
from typing import Type, cast, TYPE_CHECKING
from django.core.exceptions import ObjectDoesNotExist

from .types import Ros2SrvDef

if TYPE_CHECKING:
    from .fields import RosFieldMixin
    from .models import RosModel

logger = logging.getLogger()

#
# Generic services
#


class RosGetSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for Get"""

    def __init__(self, model: Type["RosModel"], search_field, raw):
        self.model = model
        self.search_field = search_field
        self.raw = raw

        self.name = f"Get{model.ros_msgtype if not raw else model.ros_rawmsgtype}"

        if not isinstance(self.search_field, tuple):
            self.search_field = (self.search_field,)

        if self.search_field[0] != "id":
            self.name += f"By{self.search_field[0].capitalize()}"

        self.inputs = [
            {
                "type": cast("RosFieldMixin", model._meta.get_field(field)).ros_type,
                "name": cast("RosFieldMixin", model._meta.get_field(field)).ros_name,
            }
            for field in self.search_field
        ]

        self.inputs.append(
            {
                "type": "string[]",
                "name": "fields",
                "enum": [
                    (f"FIELD_{field['name'].upper()}", field["name"])
                    for field in model.msg_fields(raw)
                ],
            }
        )

        self.outputs = [
            {"type": "bool", "name": "success"},
            {"type": "string", "name": "message"},
            {
                "type": model.ros_msgtype if not raw else model.ros_rawmsgtype,
                "name": model.ros_data_field,
            },
        ]

    def cb(self, request, response):
        try:
            query = {
                self.model._meta.get_field(field).name: getattr(
                    request,
                    cast("RosFieldMixin", self.model._meta.get_field(field)).ros_name,
                )
                for field in self.search_field
            }
            obj = self.model.objects.get(**query)
        except ObjectDoesNotExist:
            response.success = False
            response.message = "Object not found"
            return response

        response.success = True
        setattr(
            response,
            self.model.ros_data_field,
            obj.to_ros(raw=self.raw, fields=request.fields),
        )
        return response


class RosSetSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for Set"""

    def __init__(self, model, search_field, raw):
        self.model = model
        self.search_field = search_field
        self.raw = raw

        self.name = f"Set{model.ros_msgtype if not raw else model.ros_rawmsgtype}"

        if not isinstance(self.search_field, tuple):
            self.search_field = (self.search_field,)

        if self.search_field[0] != "id":
            self.name += f"By{self.search_field[0].capitalize()}"

        self.inputs = [
            {
                "type": model.ros_msgtype if not raw else model.ros_rawmsgtype,
                "name": model.ros_data_field,
            }
        ]

        self.outputs = [
            {"type": "bool", "name": "success"},
            {"type": "string", "name": "message"},
            {"type": "int64", "name": "id"},
        ]

        if "uuid" in [f.name for f in model._meta.get_fields()]:
            self.outputs.append(
                {"type": model._meta.get_field("uuid").ros_type, "name": "uuid"}
            )

    def cb(self, request, response):
        try:
            query = {
                self.model._meta.get_field(field).name: getattr(
                    getattr(request, self.model.ros_data_field),
                    self.model._meta.get_field(field).ros_name,
                )
                for field in self.search_field
            }
            obj = self.model.objects.get(**query)
        except ObjectDoesNotExist:
            if (
                self.search_field[0] == "id"
                and getattr(request, self.model.ros_data_field).id != -1
            ):
                response.success = False
                response.message = "Object not found"
                return response

            obj = self.model()

        if self.search_field[0] != "id":
            # We force id to -1 to ignore it in the set
            getattr(request, self.model.ros_data_field).id = -1

        obj.from_ros(getattr(request, self.model.ros_data_field), self.raw)

        response.success = True
        response.message = ""
        response.id = obj.id
        if "uuid" in [f.name for f in self.model._meta.get_fields()]:
            response.uuid = obj.uuid
        return response


class RosListSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for List"""

    def __init__(self, model, filter_field, raw):
        self.model = model
        self.filter_field = filter_field
        self.raw = raw

        self.name = f"List{model.ros_msgtype if not raw else model.ros_rawmsgtype }"
        self.inputs = []

        if self.filter_field:
            self.filter_field = model._meta.get_field(self.filter_field)

            self.name += f"By{self.filter_field.name.capitalize()}"
            self.inputs.append(
                {"type": self.filter_field.ros_type, "name": self.filter_field.ros_name}
            )

        self.outputs = [
            {"type": "bool", "name": "success"},
            {"type": "string", "name": "message"},
            {
                "type": (model.ros_msgtype if not raw else model.ros_rawmsgtype) + "[]",
                "name": model.ros_data_field_plural,
            },
        ]

    def cb(self, request, response):
        objs = self.model.objects.all()

        if self.filter_field:
            objs = objs.filter(
                **{self.filter_field.name: getattr(request, self.filter_field.ros_name)}
            )

        response.success = True
        response.message = ""

        setattr(
            response,
            self.model.ros_data_field_plural,
            [obj.to_ros(self.raw, thin=True) for obj in objs],
        )
        return response


class RosDeleteSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for Delete"""

    def __init__(self, model, search_field, raw):
        self.model = model
        self.search_field = search_field
        self.raw = raw

        self.name = f"Delete{model.ros_msgtype if not raw else model.ros_rawmsgtype}"

        if not isinstance(self.search_field, tuple):
            self.search_field = (self.search_field,)

        if self.search_field[0] != "id":
            self.name += f"By{self.search_field[0].capitalize()}"

        self.inputs = [
            {
                "type": model._meta.get_field(field).ros_type,
                "name": model._meta.get_field(field).ros_name,
            }
            for field in self.search_field
        ]

        self.outputs = [
            {"type": "bool", "name": "success"},
            {"type": "string", "name": "message"},
        ]

    def cb(self, request, response):
        try:
            query = {
                self.model._meta.get_field(field).name: getattr(
                    request, self.model._meta.get_field(field).ros_name
                )
                for field in self.search_field
            }
            obj = self.model.objects.get(**query)
        except ObjectDoesNotExist:
            response.success = False
            response.message = "Object not found"
            return response

        obj.delete()
        response.success = True

        return response
