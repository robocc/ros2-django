#! /usr/bin/env python3

import importlib
import re
import traceback

from django.apps import apps
from django.conf import settings

from rclpy.node import Node  # type:ignore

from .models import RosModel


class ROS2DjangoNode(Node):
    def __init__(self, django_app, node_name="ros2_django", overrides=[]):
        super().__init__(
            node_name=node_name,
            parameter_overrides=[],
            start_parameter_services=False,
        )

        itf_srv = importlib.import_module(f"{settings.ROS_INTERFACES_MODULE_NAME}.srv")

        pattern = re.compile(r"(?<!^)(?=[A-Z])")
        c = 0
        for model in apps.get_app_config(django_app).get_models():
            if not issubclass(model, RosModel):
                continue

            for srv in model.services():
                srv_path = (
                    f"/{settings.ROS_INTERFACES_MODULE_NAME}/"
                    + pattern.sub("_", srv.name).lower()
                )
                if srv_path in overrides:
                    continue
                srv_itf = getattr(itf_srv, srv.name)
                self.create_service(
                    srv_itf,
                    srv_path,
                    self.exception_wrapper(srv.cb),
                )
                c += 1

        self.get_logger().info(f"ROS2 django node started with {c} generic services")

    def exception_wrapper(self, callback):
        def wrapped(request, response):
            self.get_logger().debug(f"{callback.__name__} called with {request}")
            try:
                r = callback(request, response)
                self.get_logger().debug(f"{callback.__name__} returned {r}")
                return r
            except Exception as e:
                self.get_logger().error(
                    f"Service {callback.__name__} got an exception : {e}\n{traceback.format_exc()}"
                )
                response.success = False
                response.message = f"Got exception {e}"
                return response

        return wrapped
