# Picar Localization

## Description
This ROS package contains all code for the localization of the picar. So far this only includes the triggering and reading of sensor information from a SONAR sensor. The SONAR is mounted on top of a servo in order to turn it. The SONAR is then triggered after the servo has been moved a step. The sensor data is read, saved, and after a full 180 degree turn it is published as a laserscan message.

## Packages

<dl>
  <dt>picar_sensor_ultrasonic</dt>
  <dd>Contains code for both, moving the servo carrying the sensor and reading data from the SONAR sensor.</dd>
</dl>
