package frc.team88.swerve.module.util;

import frc.team88.swerve.module.SwerveModule;

/** A lambda function that takes a module and returns a double */
public interface ModuleDoubleSupplier {
  double getAsDouble(SwerveModule module);
}
