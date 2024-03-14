from typing import Type
import logging
from pathlib import Path
import os

from django.apps import apps
from django.core.management.base import BaseCommand

from ...models import RosModel
from ...types import Ros2MsgFieldDef, Ros2SrvDef

logger = logging.getLogger()


def field_output(ros_field: Ros2MsgFieldDef):
    out = f"{ros_field['type']} {ros_field['name']} {ros_field.get('default', '')}\n"
    if "enum" in ros_field:
        for name, val in ros_field["enum"]:
            out += f"{ros_field['type'].replace('[]', '')} {name}={val}\n"
    return out


class Command(BaseCommand):
    help = "Generate ROS msgs and srv files"

    def gen_msgs(self, model: Type[RosModel], output_path: Path):
        for raw in [False, True]:
            if raw and not model.has_raw():
                continue

            filename = (
                output_path
                / "msg"
                / ((model.ros_msgtype if not raw else model.ros_rawmsgtype) + ".msg")
            )
            with open(filename, "w") as f:
                for ros_field in model.msg_fields(raw):
                    f.write(field_output(ros_field))

    def gen_srvs(self, srv: Ros2SrvDef, output_path: Path):
        filename = output_path / "srv" / (srv.name + ".srv")

        with open(filename, "w") as f:
            for input in srv.inputs:
                f.write(field_output(input))
            f.write("---\n")
            for output in srv.outputs:
                f.write(field_output(output))

    def add_arguments(self, parser):
        parser.add_argument("django_app", type=str)
        parser.add_argument("output_dir", type=str)

    def handle(self, *args, **options):
        output_path = Path(options["output_dir"])

        os.makedirs(output_path, exist_ok=True)
        os.makedirs(output_path / "msg", exist_ok=True)
        os.makedirs(output_path / "srv", exist_ok=True)

        self.all_models = list(apps.get_app_config(options["django_app"]).get_models())
        for model in self.all_models:
            if not issubclass(model, RosModel):
                continue

            self.gen_msgs(model, output_path)
            for srv in model.services():
                self.gen_srvs(srv, output_path)
