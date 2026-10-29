import logging
from typing import Type, cast, TYPE_CHECKING
from django.core.exceptions import ObjectDoesNotExist

from .types import Ros2SrvDef, RosSearchMeta, Ros2MsgFieldDef

if TYPE_CHECKING:
    from .fields import RosFieldMixin
    from .models import RosModel

logger = logging.getLogger()

#
# Generic services
#


def handle_ros_search(
    search_field: tuple | str, model: Type["RosModel"]
) -> RosSearchMeta:
    if not isinstance(search_field, tuple):
        search_field = (search_field,)

    if search_field[0] != "id":
        label = search_field[0].capitalize()
    else:
        label = ""

    fields = []
    for field_name in search_field:
        if "__" in field_name:
            related = model
            for related_name in field_name.split("__")[:-1]:
                related = related._meta.get_field(related_name).related_model
                if related is None:
                    raise Exception(
                        f"Unable to find foreign path for field {field_name}"
                    )

            related_field_name = field_name.split("__")[-1]

            field = Ros2MsgFieldDef(
                type=cast(
                    "RosFieldMixin", related._meta.get_field(related_field_name)
                ).ros_type,
                name=cast(
                    "RosFieldMixin", related._meta.get_field(related_field_name)
                ).ros_name,
                django_filter=field_name,
            )
        else:
            field = Ros2MsgFieldDef(
                type=cast("RosFieldMixin", model._meta.get_field(field_name)).ros_type,
                name=cast("RosFieldMixin", model._meta.get_field(field_name)).ros_name,
                django_filter=cast(
                    "RosFieldMixin", model._meta.get_field(field_name)
                ).ros_name,
            )
        fields.append(field)

    return RosSearchMeta(label=label, fields=fields)


class RosGetSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for Get"""

    def __init__(self, model: Type["RosModel"], search_field, raw):
        self.model = model
        self.raw = raw

        self.name = f"Get{model.ros_msgtype if not raw else model.ros_rawmsgtype}"
        self.search_field = handle_ros_search(search_field=search_field, model=model)
        if self.search_field["label"]:
            self.name += f"By{self.search_field["label"]}"

        self.inputs = self.search_field["fields"]
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
                field["django_filter"]: getattr(request, field["name"])
                for field in self.search_field["fields"]
                if "django_filter" in field
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

    def __init__(self, model: Type["RosModel"], search_field, raw):
        self.model = model
        self.raw = raw

        self.name = f"Set{model.ros_msgtype if not raw else model.ros_rawmsgtype}"
        self.search_field = handle_ros_search(search_field=search_field, model=model)
        if self.search_field["label"]:
            self.name += f"By{self.search_field["label"]}"

        self.inputs = [
            {
                "type": model.ros_msgtype if not raw else model.ros_rawmsgtype,
                "name": model.ros_data_field,
            },
            {
                "type": "string[]",
                "name": "fields",
                "enum": [
                    (f"FIELD_{field['name'].upper()}", field["name"])
                    for field in model.msg_fields(raw)
                ],
            },
        ]

        self.outputs = [
            {"type": "bool", "name": "success"},
            {"type": "string", "name": "message"},
            {"type": "int64", "name": "id"},
        ]

        if "uuid" in [f.name for f in model._meta.get_fields()]:
            self.outputs.append(
                {"type": model._meta.get_field("uuid").ros_type, "name": "uuid"}  # type: ignore
            )

    def cb(self, request, response):
        try:
            query = {
                field["django_filter"]: getattr(request, field["name"])
                for field in self.search_field["fields"]
                if "django_filter" in field
            }
            obj = self.model.objects.get(**query)
        except ObjectDoesNotExist:
            if (
                self.search_field["fields"][0]["name"] == "id"
                and getattr(request, self.model.ros_data_field).id != -1
            ):
                response.success = False
                response.message = "Object not found"
                return response

            obj = self.model()

        if self.search_field["fields"][0]["name"] != "id":
            # We force id to -1 to ignore it in the set
            getattr(request, self.model.ros_data_field).id = -1

        obj.from_ros(
            getattr(request, self.model.ros_data_field), self.raw, fields=request.fields
        )

        response.success = True
        response.message = ""
        response.id = obj.id  # type: ignore
        if "uuid" in [f.name for f in self.model._meta.get_fields()]:
            response.uuid = obj.uuid  # type: ignore
        return response


class RosListSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for List"""

    def __init__(self, model: Type["RosModel"], filter_field, raw):
        self.model = model
        self.raw = raw

        self.name = f"List{model.ros_msgtype if not raw else model.ros_rawmsgtype }"
        if filter_field is not None:
            self.filter_field = handle_ros_search(
                search_field=filter_field, model=model
            )
            if self.filter_field["label"]:
                self.name += f"By{self.filter_field["label"]}"

            self.inputs = self.filter_field["fields"]
        else:
            self.filter_field = None
            self.inputs = []
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
                "type": (model.ros_msgtype if not raw else model.ros_rawmsgtype) + "[]",
                "name": model.ros_data_field_plural,
            },
        ]

    def cb(self, request, response):
        objs = self.model.objects.all()

        if self.filter_field:
            objs = objs.filter(
                **{
                    field["django_filter"]: getattr(request, field["name"])
                    for field in self.filter_field["fields"]
                    if "django_filter" in field
                }
            )

        response.success = True
        response.message = ""

        setattr(
            response,
            self.model.ros_data_field_plural,
            [obj.to_ros(self.raw, thin=True, fields=request.fields) for obj in objs],
        )
        return response


class RosDeleteSrv(Ros2SrvDef):
    """Service definition (name, inputs, output and callback) for Delete"""

    def __init__(self, model: Type["RosModel"], search_field, raw):
        self.model = model
        self.raw = raw

        self.name = f"Delete{model.ros_msgtype if not raw else model.ros_rawmsgtype}"
        self.search_field = handle_ros_search(search_field=search_field, model=model)
        if self.search_field["label"]:
            self.name += f"By{self.search_field["label"]}"

        self.inputs = self.search_field["fields"]
        self.outputs = [
            {"type": "bool", "name": "success"},
            {"type": "string", "name": "message"},
        ]

    def cb(self, request, response):
        try:
            query = {
                field["django_filter"]: getattr(request, field["name"])
                for field in self.search_field["fields"]
                if "django_filter" in field
            }
            obj = self.model.objects.get(**query)
        except ObjectDoesNotExist:
            response.success = False
            response.message = "Object not found"
            return response

        obj.delete()
        response.success = True

        return response
