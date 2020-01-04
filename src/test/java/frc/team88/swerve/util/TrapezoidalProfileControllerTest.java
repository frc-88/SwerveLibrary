package frc.team88.swerve.util;

import org.junit.jupiter.api.BeforeEach;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.wrappers.RobotControllerWrapper;

public class TrapezoidalProfileControllerTest {

     @Mock
     private SyncPIDController positionPID;

     @Mock
     private RobotControllerWrapper robotController;

     private TrapezoidalProfileController controller;

     private final double MAX_SPEED = 10;
     private final double MAX_ACCELERATION = 100;

     @BeforeEach
     public void setup() {
        MockitoAnnotations.initMocks(this);
        RobotControllerWrapper.setInstance(robotController);

        this.controller = new TrapezoidalProfileController(MAX_SPEED, MAX_ACCELERATION, positionPID);
     }

     

}
