package frc.team88.swerve.kinematics;

import static org.mockito.Mockito.when;

import static frc.team88.swerve.TestUtils.assertDoubleEquals;
import static frc.team88.swerve.TestUtils.assertVectorEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.kinematics.InverseKinematics;
import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

public class InverseKinematicsTest {

    private InverseKinematics ik;

    @Mock
    private SwerveModule module1;

    @Mock
    private SwerveModule module2;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);

        ik = new InverseKinematics(module1, module2);
    }

    @Test
    public void testCalculateModuleTranslationVector() {
        Vector2D translation = Vector2D.createPolarCoordinates(2, new WrappedAngle(90));
        Vector2D result = ik.calculateModuleTranslationVector(
                FullVelocityMotionState.createRobotCentricDefault().changeTranslationVelocity(translation));
        assertVectorEquals(translation, result);
    }

    @Test
    public void testCalculateModuleRotationVectorPositive() {
        Vector2D location = Vector2D.createCartesianCoordinates(3, 2);
        when(module1.getLocation()).thenReturn(location);
        Vector2D result = ik.calculateModuleRotationVectors(FullVelocityMotionState.createRobotCentricDefault()
                .changeRotationVelocity(180).changeCenterOfRotation(Vector2D.createCartesianCoordinates(1, 0)),
                module1);
        assertDoubleEquals(180. * Math.sqrt(8.) * (2. * Math.PI) / 360., result.getMagnitude());
        assertDoubleEquals(45., result.getAngle().asDouble());
    }

    @Test
    public void testCalculateModuleRotationVectorNegative() {
        Vector2D location = Vector2D.createCartesianCoordinates(1, -2);
        when(module1.getLocation()).thenReturn(location);
        Vector2D result = ik.calculateModuleRotationVectors(FullVelocityMotionState.createRobotCentricDefault().changeRotationVelocity(-90), module1);

        assertDoubleEquals(90. * Math.sqrt(5.) * (2. * Math.PI) / 360., result.getMagnitude());
        assertDoubleEquals(location.getAngle().plus(-90).asDouble(), result.getAngle().asDouble());
    }

    @Test
    public void testScaleIntoRangeNoScaling() {
        Vector2D[] input = { Vector2D.createPolarCoordinates(10, new WrappedAngle(0)),
                Vector2D.createPolarCoordinates(8, new WrappedAngle(60)),
                Vector2D.createPolarCoordinates(10, new WrappedAngle(-90)),
                Vector2D.createPolarCoordinates(4, new WrappedAngle(70)) };
        ik.setMaxSpeed(10);
        Vector2D[] output = ik.scaleIntoRange(input);

        assertDoubleEquals(10, output[0].getMagnitude());
        assertDoubleEquals(0, output[0].getAngle().asDouble());
        assertDoubleEquals(8, output[1].getMagnitude());
        assertDoubleEquals(60, output[1].getAngle().asDouble());
        assertDoubleEquals(10, output[2].getMagnitude());
        assertDoubleEquals(-90, output[2].getAngle().asDouble());
        assertDoubleEquals(4, output[3].getMagnitude());
        assertDoubleEquals(70, output[3].getAngle().asDouble());
    }

    @Test
    public void testScaleIntoRangeScaleFromFirst() {
        Vector2D[] input = { Vector2D.createPolarCoordinates(20, new WrappedAngle(0)),
                Vector2D.createPolarCoordinates(8, new WrappedAngle(60)),
                Vector2D.createPolarCoordinates(16, new WrappedAngle(-90)),
                Vector2D.createPolarCoordinates(4, new WrappedAngle(70)) };
        ik.setMaxSpeed(10);
        Vector2D[] output = ik.scaleIntoRange(input);

        assertDoubleEquals(10, output[0].getMagnitude());
        assertDoubleEquals(0, output[0].getAngle().asDouble());
        assertDoubleEquals(4, output[1].getMagnitude());
        assertDoubleEquals(60, output[1].getAngle().asDouble());
        assertDoubleEquals(8, output[2].getMagnitude());
        assertDoubleEquals(-90, output[2].getAngle().asDouble());
        assertDoubleEquals(2, output[3].getMagnitude());
        assertDoubleEquals(70, output[3].getAngle().asDouble());
    }

    @Test
    public void testScaleIntoRangeScaleFromMiddle() {
        Vector2D[] input = { Vector2D.createPolarCoordinates(20, new WrappedAngle(0)),
                Vector2D.createPolarCoordinates(16, new WrappedAngle(60)),
                Vector2D.createPolarCoordinates(40, new WrappedAngle(-90)),
                Vector2D.createPolarCoordinates(12, new WrappedAngle(70)) };
        ik.setMaxSpeed(10);
        Vector2D[] output = ik.scaleIntoRange(input);

        assertDoubleEquals(5, output[0].getMagnitude());
        assertDoubleEquals(0, output[0].getAngle().asDouble());
        assertDoubleEquals(4, output[1].getMagnitude());
        assertDoubleEquals(60, output[1].getAngle().asDouble());
        assertDoubleEquals(10, output[2].getMagnitude());
        assertDoubleEquals(-90, output[2].getAngle().asDouble());
        assertDoubleEquals(3, output[3].getMagnitude());
        assertDoubleEquals(70, output[3].getAngle().asDouble());
    }
}
