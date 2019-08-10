package frc.team88.swerve.state;

import frc.team88.swerve.util.Vector2D;

public class AbsoluteHeadingSwerveState extends VelocitySwerveState {

    public static final AbsoluteHeadingSwerveState ZERO_STATE = 
        new AbsoluteHeadingSwerveState(0, Vector2D.ORIGIN, 0);

    private final double heading;

    public AbsoluteHeadingSwerveState(double heading, Vector2D translationVelocity, 
            double rotationalVelocity) {

        super(translationVelocity, rotationalVelocity);
        this.heading = heading;
    }

    public double getHeading() {
        return this.heading;
    }

    public AbsoluteHeadingSwerveState rotateFrame(double angle) {
        VelocitySwerveState vss = super.rotateFrame(angle);
        return new AbsoluteHeadingSwerveState(heading + angle, vss.getTranslationVelocity(), 
            vss.getRotationalVelocity());
    }

    @Override
    public <R> R accept(SwerveStateVisitor<R> v) {
        return v.visitAbsoluteHeadingSwerveState(this);
    }

}