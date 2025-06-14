:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/camera_sensor_broadcaster/doc/userdoc.rst

.. _camera_sensor_broadcaster_userdoc:

Camera Sensor Broadcaster
--------------------------------
Broadcaster of messages from Camera sensor.
The published message type is ``sensor_msgs/msg/Image``.

The controller is a wrapper around ``CameraSensor`` semantic component (see ``controller_interface`` package).

Parameters
^^^^^^^^^^^
The Camera Sensor Broadcaster uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/camera_sensor_broadcaster/src/camera_sensor_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.


List of parameters
=========================
.. generate_parameter_library_details:: ../src/camera_sensor_broadcaster_parameters.yaml


An example parameter file
=========================

.. generate_parameter_library_default::
  ../src/camera_sensor_broadcaster_parameters.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/camera_sensor_broadcaster/test/camera_sensor_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/camera_sensor_broadcaster_params.yaml
   :language: yaml
