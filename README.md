# ros2-django

## Introduction

![Bring django power into your ROS2 worskpace](./img/logo.png)

This project is a Django application that aims to be integrated in a ROS2 python package.

Your ROS2 package will then be able to start a node that will export CRUD (Create, Read, Update, Delete) ROS services
for Django models.

The goal is to have a way to easily store and retrieve structured data in a ROS2 ecosystem via services, with the
power of django framework that includes schema migration, admin interface, field validation, and many more.

## Installation

For now, install it directly from GitHub

```
pip install https://github.com/robocc/ros2-django.git
```

Next, create a ROS2 package following the example in `example_pkg/`

## Usage

### Package creation using boilerplate

The quickest way is to copy/use `example_pkg` in your workspace. It contains a basic django app with 2 ros models and ros2-django configured.

```sh
cd example_pkg
colcon build

./manage.py migrate  # create database
./manage.py createsuperuser # create admin user

./example_pkg_node.py  # start ros node
```

You can also run

```sh
./manage.py runserver
```
and connect to http://localhost:8000 to access admin interface

### models.py

From now, you can edit your application `models.py` and create your model objects. 

- All the models you want to export to ROS should inherit from `ros2_django.models.RosModel`
- All the fields you want to export to ROS should be from module `ros2_django.fields` (for example `ros2_django.fields.RosCharField`)
- A specific `ros2_django.fields.RosMsgField` allows you to store/retrieve directly a ROS message by specifying its type
- Custom fields can be created, they should inherit from both `ros2_django.fields.RosFieldMixin` and correspondant Django field

Consider the following example of `models.py`
```python
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


class PoseOnMap(RosModel):
    id = ros2_django.fields.RosBigAutoField(primary_key=True)
    name = ros2_django.fields.RosCharField(max_length=128)
    pose = ros2_django.fields.RosMsgField(ros_msg=Pose, ros_type="geometry_msgs/Pose")
    map = ros2_django.fields.RosForeignKey(
        "Map",
        on_delete=models.CASCADE,
    )
```
In this example, `Map` and `PoseOnMap` messages will be created, with all the fields. And the `pose` field of `PoseOnMap` will be a standard `geometry_msgs/Pose`.


### Generating interfaces

*This will be done automatically with the example CMakeLists.txt*

Once `models.py` is written, you can create your database with standard `./manage.py makemigrations` and `./manage.py migrate`.

You now also can run `./manage.py gen_ros_msgs example_app out/` that will generate all messages and services definition for ROS in directory `out/`.

For each `RosModel` named `Foo`, it will generate:
 - `Foo.msg` with all its fields, including reverse relations
 - `FooRaw.msg` with all its fields without relations (not generated if its the same as `Foo.msg`)
 - `GetFoo.srv` and `GetFooRaw.srv` services to get a `Foo` or `FooRaw` via its id
 - `SetFoo.srv` and `SetFooRaw.srv` to create/set a `Foo` or `FooRaw` object via its id (creation with `id=-1`)
 - `ListFoo.srv` and `ListFooRaw.srv` to list all `Foo` or `FooRaw`
 - `DeleteFoo.srv` and `DeleteFooRaw.srv` to remove a `Foo` or `FooRaw` via its id

Theses services will be implemented and served via `ros2_django.ros_node.ROS2DjangoNode`

Example interfaces for previous model example:

`Map.msg`:

```
PoseOnMap[] poseonmap
int64 id
string name
string description
uint8[] webp
```

`MapRaw.msg`:

```
int64 id
string name
string description
uint8[] webp
```

`PoseOnMap.msg`:

```
int64 id 
string name 
geometry_msgs/Pose pose 
int64 id_map 
```

`GetMap.srv`:

```
int64 id
---
bool success
string message
Map map
```

`SetMapRaw.srv`:

```
MapRaw map
---
bool success
string message
int64 id
```

`ListMap.srv`:

```
---
bool success
string message
Map[] maps
```


### Custom interfaces behaviour

In order to customize interface services, one can declare a `RosMeta` class inside model.

Current accepted attributes are:

 - `ros_search`: list of fields (or fields tuple for and search) that we can search upon for `Get` and `Set` services.
   - Following example will generate a `GetFooByName` and `SetFooByName` service
    ```py
     class Foo(RosModel):
        id = ros2_django.fields.RosBigAutoField(primary_key=True)
        name = ros2_django.fields.RosCharField(max_length=128)
        
        class RosMeta:
            ros_search = ['name']
    ```
    Generated `GetFooByName.srv`
    ```
    string name
    ---
    bool success
    string message
    Foo  foo
    ```
 - `ros_filter`: list of fields to filter upon for `List` services
    - Following example will generate a `ListBarByFoo` to list all `Bar` that depends on a specific `Foo`
    ```py
    class Bar(RosModel):
        id = ros2_django.fields.RosBigAutoField(primary_key=True)
        foo = ros2_django.fields.RosForeignKey(
            "Foo",
            on_delete=models.CASCADE,
        )
        
        class RosMeta:
            ros_filter = ['foo']
    ```
    Generated `ListBarByFoo.srv`
    ```
    int64 id_foo
    ---
    bool success
    string message
    Bar[] bars
    ```
 - `ros_ignore_fields`: list of fields not included in the final message
 - `ros_thin_fields`: if present, will restrict which fields will be filled in `List` services to gain bandwidth

# Advanced

### Package creation

*`example_pkg` folder contains a ready-to-use package using `ros2-django`. These instructions are for people who want to do it manually*

 - Create a ROS2 package with a `CMakeLists.txt` (*not* a Python package)
   - `ros2 pkg create --build-type ament_cmake example_pkg`
 - Create a standard Django project and app in your package
   - `django-admin startproject example_django_project .`
   - `./manage.py startapp example_app`
 - In `example_django_project/settings.py`
   - Add `ros2_django` in `INSTALLED_APPS`
   - Set `ROS_INTERFACES_MODULE_NAME` to ROS package name (`example_pkg` in this case)
 - Setup routes/admin stuff for Django if you want (cf. django documentation)
 - Edit your `CMakeLists.txt` to generate interfaces and install python packages (cf. example)
 - Create a node starter python file that will spin `ros2_django.ros_node.ROS2DjangoNode` (cf. example)