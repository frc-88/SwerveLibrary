package frc.team88.swerve.motion.kinematics;

import static frc.team88.swerve.TestUtils.assertDoubleEquals;
import static frc.team88.swerve.TestUtils.assertVectorEquals;

import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class InverseKinematicsTest {

  private InverseKinematics ik;

  @Mock private SwerveModule module1;

  @Mock private SwerveModule module2;

  @BeforeEach
  public void setup() {
    MockitoAnnotations.initMocks(this);

    ik = new InverseKinematics(module1, module2);
  }

  @Test
  public void testCalculateModuleTranslationVector() {
    Vector2D translation = Vector2D.createPolarCoordinates(2, new WrappedAngle(90));
    Vector2D result =
        ik.calculateModuleTranslationVector(
            new VelocityState(
                translation.getAngle().asDouble(), translation.getMagnitude(), 0, false));
    assertVectorEquals(translation, result);
  }

  @Test
  public void testCalculateModuleRotationVectorPositive() {
    Vector2D location = Vector2D.createCartesianCoordinates(3, 2);
    Vector2D result =
        ik.calculateRotationVector(new VelocityState(0, 0, 180, 1, 0, false), location);
    assertDoubleEquals(180. * Math.sqrt(8.) * (2. * Math.PI) / 360., result.getMagnitude());
    assertDoubleEquals(135., result.getAngle().asDouble());
  }

  @Test
  public void testCalculateModuleRotationVectorNegative() {
    Vector2D location = Vector2D.createCartesianCoordinates(1, -2);
    Vector2D result =
        ik.calculateRotationVector(new VelocityState(0, 0, -90, false), location);

    assertDoubleEquals(90. * Math.sqrt(5.) * (2. * Math.PI) / 360., result.getMagnitude());
    assertDoubleEquals(location.getAngle().plus(-90).asDouble(), result.getAngle().asDouble());
  }
}
