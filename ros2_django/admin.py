import base64

from django import forms
from django.contrib import admin

from .fields import RosImageField, RosMsgField


class ImageFileInput(forms.ClearableFileInput):
    template_name = "binary_image_file_input.html"

    def is_initial(self, value):
        return bool(value)

    def format_value(self, value):
        if self.is_initial(value):
            if isinstance(value, str):
                return value
            return base64.b64encode(value).decode()

    def value_from_datadict(self, data, files, name):
        upload = super().value_from_datadict(data, files, name)
        if upload is None:
            return data.get(f"{name}_data")
        if not upload:
            return base64.b64encode(b"").decode()
        return base64.b64encode(upload.read()).decode()


class RosMsgInput(forms.Textarea):
    def value_from_datadict(self, data, files, name):
        return super().value_from_datadict(data, files, name)


class ROS2DjangoModelAdmin(admin.ModelAdmin):
    formfield_overrides = {
        RosImageField: {"widget": ImageFileInput()},
        RosMsgField: {"widget": RosMsgInput()},
    }
