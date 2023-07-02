from django.db import models
from ros2_django.models import RosModel
import ros2_django.fields
from geometry_msgs.msg import Pose


class Map(RosModel):
    id = ros2_django.fields.RosBigAutoField(primary_key=True)
    name = ros2_django.fields.RosCharField(max_length=128)
    description = ros2_django.fields.RosTextField(blank=True)
    webp = ros2_django.fields.RosImageField(blank=True, null=True)

    def __str__(self):
        return f"[{self.id}] {self.name}"

    class RosMeta:
        ros_thin_fields = ["id", "name", "description"]


class PoseOnMap(RosModel):
    id = ros2_django.fields.RosBigAutoField(primary_key=True)
    name = ros2_django.fields.RosCharField(max_length=128)
    pose = ros2_django.fields.RosMsgField(ros_msg=Pose, ros_type="geometry_msgs/Pose")
    map = ros2_django.fields.RosForeignKey(
        "Map",
        on_delete=models.CASCADE,
    )

    def __str__(self):
        return f"Pose {self.name} @ {self.map}"

    class RosMeta:
        ros_filter = ["map"]
