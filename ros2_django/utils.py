from array import array
import json
import importlib

from rclpy.type_support import check_is_valid_msg_type  # type:ignore


def ros2json(ros_obj):
    def ros2obj(ros_obj):
        out = {}
        for field in ros_obj.get_fields_and_field_types().keys():
            value = getattr(ros_obj, field)
            try:
                check_is_valid_msg_type(value)
                ros_field = True
            except AttributeError:
                ros_field = False

            if ros_field:
                out[field] = ros2obj(value)
            elif isinstance(value, list) or isinstance(value, array):
                out[field] = [ros2obj(e) for e in value]
            else:
                out[field] = value
        return out

    return json.dumps(ros2obj(ros_obj))


def convert_type(val, ros_type):
    """Cf. https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#field-types"""

    if ros_type == "bool":
        return bool(val)
    elif ros_type == "byte":
        return bytes(val)
    elif ros_type in ("float32", "float64", "double"):
        return float(val)
    elif ros_type in (
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
    ):
        return int(val)
    elif ros_type in ("string", "wstring", "char"):
        return str(val)
    else:
        raise Exception(f"Unknown ros type {ros_type} !!")


def json2ros(json_obj_s, ros_class):
    def obj2ros(json_obj, ros_class):
        out = ros_class()
        for i, (field, field_type) in enumerate(
            ros_class.get_fields_and_field_types().items()
        ):
            value = getattr(out, field)
            try:
                check_is_valid_msg_type(value)
                ros_field = True
            except AttributeError:
                ros_field = False

            if ros_field and field in json_obj:
                setattr(out, field, obj2ros(json_obj[field], type(value)))
            elif isinstance(value, list):
                # Horrible but only way to get list type of ROS obj
                stype = ros_class.SLOT_TYPES[i].value_type
                ltype = getattr(
                    importlib.import_module(".".join(stype.namespaces)), stype.name
                )
                setattr(out, field, [obj2ros(e, ltype) for e in json_obj[field]])
            elif field in json_obj:
                setattr(out, field, convert_type(json_obj[field], field_type))

        return out

    return obj2ros(json.loads(json_obj_s), ros_class)
