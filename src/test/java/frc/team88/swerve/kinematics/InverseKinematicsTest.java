package frc.team88.swerve.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.swervemodule.SwerveModule;
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
        ik.setTranslationVelocity(translation);
        Vector2D result = ik.calculateModuleTranslationVector();
        assertTrue(translation.approximatelyEquals(result));
    }

    @Test
    public void testCalculateModuleRotationVectorPositive() {
        Vector2D location = Vector2D.createCartesianCoordinates(3, 2);
        when(module1.getLocation()).thenReturn(location);
        ik.setRotationVelocity(180);
        ik.setCenterOfRotation(Vector2D.createCartesianCoordinates(1, 0));
        Vector2D result = ik.calculateModuleRotationVectors(module1);

        assertEquals(180. * Math.sqrt(8.) * (2. * Math.PI) / 360., result.getMagnitude(), 0.000001);
        assertEquals(45., result.getAngle().asDouble(), 0.000001);
    }

    @Test
    public void testCalculateModuleRotationVectorNegative() {
        Vector2D location = Vector2D.createCartesianCoordinates(1, -2);
        when(module1.getLocation()).thenReturn(location);
        ik.setRotationVelocity(-90);
        ik.setCenterOfRotation(Vector2D.createCartesianCoordinates(0, 0));
        Vector2D result = ik.calculateModuleRotationVectors(module1);

        assertEquals(90. * Math.sqrt(5.) * (2. * Math.PI) / 360., result.getMagnitude(), 0.000001);
        assertEquals(location.getAngle().plus(-90).asDouble(), result.getAngle().asDouble(), 0.000001);
    }

    @Test
    public void testScaleIntoRangeNoScaling() {
        Vector2D[] input = {
            Vector2D.createPolarCoordinates(10, new WrappedAngle(0)),
            Vector2D.createPolarCoordinates(8, new WrappedAngle(60)),
            Vector2D.createPolarCoordinates(10, new WrappedAngle(-90)),
            Vector2D.createPolarCoordinates(4, new WrappedAngle(70))
        };
        ik.setMaxSpeed(10);
        Vector2D[] output = ik.scaleIntoRange(input);

        assertEquals(10, output[0].getMagnitude(), 0.000001);
        assertEquals(0, output[0].getAngle().asDouble(), 0.000001);
        assertEquals(8, output[1].getMagnitude(), 0.000001);
        assertEquals(60, output[1].getAngle().asDouble(), 0.000001);
        assertEquals(10, output[2].getMagnitude(), 0.000001);
        assertEquals(-90, output[2].getAngle().asDouble(), 0.000001);
        assertEquals(4, output[3].getMagnitude(), 0.000001);
        assertEquals(70, output[3].getAngle().asDouble(), 0.000001);
    }

    @Test
    public void testScaleIntoRangeScaleFromFirst() {
        Vector2D[] input = {
            Vector2D.createPolarCoordinates(20, new WrappedAngle(0)),
            Vector2D.createPolarCoordinates(8, new WrappedAngle(60)),
            Vector2D.createPolarCoordinates(16, new WrappedAngle(-90)),
            Vector2D.createPolarCoordinates(4, new WrappedAngle(70))
        };
        ik.setMaxSpeed(10);
        Vector2D[] output = ik.scaleIntoRange(input);

        assertEquals(10, output[0].getMagnitude(), 0.000001);
        assertEquals(0, output[0].getAngle().asDouble(), 0.000001);
        assertEquals(4, output[1].getMagnitude(), 0.000001);
        assertEquals(60, output[1].getAngle().asDouble(), 0.000001);
        assertEquals(8, output[2].getMagnitude(), 0.000001);
        assertEquals(-90, output[2].getAngle().asDouble(), 0.000001);
        assertEquals(2, output[3].getMagnitude(), 0.000001);
        assertEquals(70, output[3].getAngle().asDouble(), 0.000001);
    }

    @Test
    public void testScaleIntoRangeScaleFromMiddle() {
        Vector2D[] input = {
            Vector2D.createPolarCoordinates(20, new WrappedAngle(0)),
            Vector2D.createPolarCoordinates(16, new WrappedAngle(60)),
            Vector2D.createPolarCoordinates(40, new WrappedAngle(-90)),
            Vector2D.createPolarCoordinates(12, new WrappedAngle(70))
        };
        ik.setMaxSpeed(10);
        Vector2D[] output = ik.scaleIntoRange(input);

        assertEquals(5, output[0].getMagnitude(), 0.000001);
        assertEquals(0, output[0].getAngle().asDouble(), 0.000001);
        assertEquals(4, output[1].getMagnitude(), 0.000001);
        assertEquals(60, output[1].getAngle().asDouble(), 0.000001);
        assertEquals(10, output[2].getMagnitude(), 0.000001);
        assertEquals(-90, output[2].getAngle().asDouble(), 0.000001);
        assertEquals(3, output[3].getMagnitude(), 0.000001);
        assertEquals(70, output[3].getAngle().asDouble(), 0.000001);
    }
}