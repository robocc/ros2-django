from django.contrib import admin

from .models import Map, PoseOnMap
from ros2_django.admin import ROS2DjangoModelAdmin

admin.site.register(Map, ROS2DjangoModelAdmin)
admin.site.register(PoseOnMap, ROS2DjangoModelAdmin)
