# panther_controller

## Changing Wheel Type

Changing wheel types is possible and can be done for both the real robot and the simulation. By default, three types of wheels are supported using the launch argument `wheel_type`. If you want to use custom wheels, all you need to do is point to the new wheel and controller configuration files using the `wheel_config_path` and `controller_config_path` parameters. These files should be based on the default files, i.e. [WH01_controller.yaml](./config/WH01_controller.yaml) and [WH01.yaml](../panther_description/config/WH01.yaml).
