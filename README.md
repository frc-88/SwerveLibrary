[![Actions Status](https://github.com/frc-88/SwerveLibrary/workflows/Java%20CI/badge.svg)](https://github.com/frc-88/SwerveLibrary/actions) [![Maven Central](https://maven-badges.herokuapp.com/maven-central/{io.github.frc-88}/{swerve}/badge.svg)](https://maven-badges.herokuapp.com/maven-central/io.gitub.frc-88/swerve) [![javadoc](https://javadoc.io/badge2/io.github.frc-88/swerve/javadoc.svg)](https://javadoc.io/doc/io.github.frc-88/swerve)


# SwerveLibrary

Swerve controls code developed by FRC Team 88 TJ². Capable of controlling both standard and differential swerves.

SwerveLibrary is the officially supported software library for TJ²'s Diff Swerve 2021.

Intended for use with the [wpilib](https://docs.wpilib.org/en/stable/) Java framework.

## Installation

To install, simply add the following line to the dependencies section of build.gradle

```groovy
dependencies {
    // WPILib autofills this section with some code. Do not delete it.
    
    compile io.github.frc-88:swerve:0.1.0
}
```

## Usage

SwerveLibrary makes use of an extensive configuration scheme for describing the robot setup. Below
is an example of a minimal config file.

#### **`swerve.toml`**
```python
[gyro]
template = "navx" # The type of gyro being used.
port-type = "I2C" # The type of port the navx is plugged into.
port = "kMXP" # The name of the port, as passed into the navx constructor.

[[modules]]
template = "team88.diff-swerve-2021.beta" # The template for the module.
location-inches = { x = 10.991, y = 12.491 } # The location of the module.
motors.lo.can-id = 0 # The CAN ID of the first motor.
motors.hi.can-id = 1 # The CAN ID of the second motor.
azimuth-sensor.can-id = 0 # The CAN ID of the CANCoder.
azimuth-sensor.offset = 0 # The degrees to subtract from the read module angle.

[[modules]]
template = "team88.diff-swerve-2021.beta" # The template for the module.
location-inches = { x = -10.991, y = 12.491 } # The location of the module.
motors.lo.can-id = 2 # The CAN ID of the first motor.
motors.hi.can-id = 3 # The CAN ID of the second motor.
azimuth-sensor.can-id = 1 # The CAN ID of the CANCoder.
azimuth-sensor.offset = 0 # The degrees to subtract from the read module angle.

[[modules]]
template = "team88.diff-swerve-2021.beta" # The template for the module.
location-inches = { x = -10.991, y = -12.491 } # The location of the module.
motors.lo.can-id = 12 # The CAN ID of the first motor.
motors.hi.can-id = 13 # The CAN ID of the second motor.
azimuth-sensor.can-id = 2 # The CAN ID of the CANCoder.
azimuth-sensor.offset = 0 # The degrees to subtract from the read module angle.

[[modules]]
template = "team88.diff-swerve-2021.beta" # The template for the module.
location-inches = { x = 10.991, y = -12.491 } # The location of the module.
motors.lo.can-id = 14 # The CAN ID of the first motor.
motors.hi.can-id = 15 # The CAN ID of the second motor.
azimuth-sensor.can-id = 3 # The CAN ID of the CANCoder.
azimuth-sensor.offset = 0 # The degrees to subtract from the read module angle.
```

Place this file in the `src/main/deploy/` directory of your robot project, so it will upload
to the RoboRIO whenever code is deployed. 

The API for this library is provided through the [SwerveController](https://www.javadoc.io/doc/io.github.frc-88/swerve/latest/frc/team88/swerve/SwerveController.html) 
class, which is constructed with the path to the config file in the deploy directory.

```java
import frc.team88.swerve.SwerveController;

// ...

SwerveController swerveController = new SwerveController("swerve.toml");
```

## Documentation

See [the wiki page](https://github.com/frc-88/SwerveLibrary/wiki) for further usage documentation.

Javadocs for this project can be found [here](https://www.javadoc.io/doc/io.github.frc-88/swerve/latest/frc/team88/swerve/SwerveController.html).

## Examples

Example projects using this library can be found in [examples/](https://github.com/frc-88/SwerveLibrary/tree/master/examples)

## Support

For support, contact Paul Terrasi at paul.a.terrasi@gmail.com