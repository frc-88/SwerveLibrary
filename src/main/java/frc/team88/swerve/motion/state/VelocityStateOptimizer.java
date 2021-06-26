package frc.team88.swerve.motion.state;

import java.util.Iterator;

/**
 * Takes a base velocity state and a desired velocity state, and tries to get
 * as close to the desired velocity state as possible. It takes the total
 * required change to get from the base to the desired state as
 * <transDirection, transSpeed, rotationVel, cOfRX, cOfRY> and does a binary
 * search between that and 0 to the desired precision, with all values being
 * scaled proportionally to each other.
 */
public class VelocityStateOptimizer implements Iterator<VelocityState> {

    private final VelocityState baseState;
    private final double[] desiredChange;
    private final double precision;

    /**
     * Creates a velocity state optimizer.
     * 
     * @param baseState The baseline state to serve as a bound on our search.
     * @param desiredState
     * @param precision
     */
    public VelocityStateOptimizer(VelocityState baseState, VelocityState desiredState, double precision) {

    }

    @Override
    public boolean hasNext() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public VelocityState next() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
