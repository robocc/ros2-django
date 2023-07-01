import json
import importlib

from rclpy.type_support import check_is_valid_msg_type


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
            elif isinstance(value, list):
                out[field] = [ros2obj(e) for e in value]
            else:
                out[field] = value
        return out

    return json.dumps(ros2obj(ros_obj))


def json2ros(json_obj_s, ros_class):
    def obj2ros(json_obj, ros_class):
        out = ros_class()
        for i, field in enumerate(ros_class.get_fields_and_field_types().keys()):
            value = getattr(out, field)
            try:
                check_is_valid_msg_type(value)
                ros_field = True
            except AttributeError:
                ros_field = False

            if ros_field:
                setattr(out, field, obj2ros(json_obj[field], type(value)))
            elif isinstance(value, list):
                # Horrible but only way to get list type of ROS obj
                stype = ros_class.SLOT_TYPES[i].value_type
                ltype = getattr(
                    importlib.import_module(".".join(stype.namespaces)), stype.name
                )
                setattr(out, field, [obj2ros(e, ltype) for e in json_obj[field]])
            else:
                setattr(out, field, json_obj[field])
        return out

    return obj2ros(json.loads(json_obj_s), ros_class)
