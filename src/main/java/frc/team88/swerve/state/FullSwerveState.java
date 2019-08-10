package frc.team88.swerve.state;

import frc.team88.swerve.util.Vector2D;

public class FullSwerveState extends AbsoluteHeadingSwerveState {

    public static final FullSwerveState ZERO_STATE = 
        new FullSwerveState(Vector2D.ORIGIN, 0, Vector2D.ORIGIN, 0);

    private final Vector2D position;

    public FullSwerveState(Vector2D position, double heading, 
            Vector2D translationVelocity, double rotationalVelocity) {

        super(heading, translationVelocity, rotationalVelocity);
        this.position = position;
    }

    public Vector2D getPosition() {
        return this.position;
    }

    public FullSwerveState rotate(double angle) {
        AbsoluteHeadingSwerveState ahss = super.rotateFrame(angle);
        return new FullSwerveState(position.rotate(angle), ahss.getHeading(), 
            ahss.getTranslationVelocity(), ahss.getRotationalVelocity());
    }

    @Override
    public <R> R accept(SwerveStateVisitor<R> v) {
        return v.visitFullSwerveState(this);
    }

}