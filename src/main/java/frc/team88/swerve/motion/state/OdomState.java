package frc.team88.swerve.motion.state;

public class OdomState {
    public double x = 0.0;
    public double y = 0.0;
    public double t = 0.0;

    public double vx = 0.0;
    public double vy = 0.0;
    public double vt = 0.0;
    
    public OdomState()
    {

    }

    public OdomState(double x0, double y0)
    {
        x = x0;
        y = y0;
    }
}
