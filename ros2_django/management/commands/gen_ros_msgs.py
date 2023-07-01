from typing import Type
import logging
import os

from django.apps import apps
from django.core.management.base import BaseCommand
from django.conf import settings

from ...models import RosModel
from ...services import RosSrv

logger = logging.getLogger()


class Command(BaseCommand):
    help = "Generate ROS msgs and srv files"

    def gen_msgs(self, model: Type[RosModel]):
        for raw in [False, True]:
            if raw and not model.has_raw():
                continue

            filename = (
                settings.ROS_INTERFACES_PATH
                / "msg"
                / ((model.ros_msgtype if not raw else model.ros_rawmsgtype) + ".msg")
            )
            with open(filename, "w") as f:
                for ros_field in model.msg_fields(raw):
                    f.write(
                        f"{ros_field['type']} {ros_field['name']} {ros_field.get('default', '')}\n"
                    )

    def gen_srvs(self, srv: RosSrv):
        filename = settings.ROS_INTERFACES_PATH / "srv" / (srv.name + ".srv")

        if os.path.isfile(
            settings.ROS_INTERFACES_PATH / ".." / "srv" / (srv.name + ".srv")
        ):
            return

        with open(filename, "w") as f:
            for input in srv.inputs:
                f.write(f"{input['type']} {input['name']}\n")
            f.write("---\n")
            for input in srv.outputs:
                f.write(f"{input['type']} {input['name']}\n")

    def handle(self, *args, **options):
        os.makedirs(settings.ROS_INTERFACES_PATH, exist_ok=True)
        os.makedirs(settings.ROS_INTERFACES_PATH / "msg", exist_ok=True)
        os.makedirs(settings.ROS_INTERFACES_PATH / "srv", exist_ok=True)

        self.all_models = list(apps.get_app_config("app").get_models())
        for model in self.all_models:
            if not issubclass(model, RosModel):
                continue

            self.gen_msgs(model)
            for srv in model.services():
                self.gen_srvs(srv)
