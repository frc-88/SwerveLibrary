package frc.team88.swerve.util;

public class Vector2D {

    public static Vector2D ORIGIN = createCartesianCoordinates(0, 0);

    private final double x;
    private final double y;

    private Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public double getAngle() {
        return Math.toDegrees(Math.atan2(-x, y));
    }

    public static Vector2D createCartesianCoordinates(double x, double y) {
        return new Vector2D(x, y);
    }

    public static Vector2D createPolarCoordinates(double magnitude, double angle) {
        angle = Math.toRadians(angle);
        return createCartesianCoordinates(
            magnitude * -Math.sin(angle), magnitude * Math.cos(angle));
    }

    public Vector2D plus(Vector2D that) {
        return createCartesianCoordinates(this.getX() + that.getX(), this.getY() + that.getY());
    }

    public Vector2D times(double scalar) {
        return createCartesianCoordinates(this.getX() * scalar, this.getY() * scalar);
    }

    public Vector2D rotate(double angle) {
        return createPolarCoordinates(this.getMagnitude(), this.getAngle() + angle);
    }

    public Vector2D normal() {
        return this.rotate(90);
    }

    public static Vector2D findIntersection(Vector2D v1Start, Vector2D v1End, Vector2D v2Start, Vector2D v2End) {

         // TODO

        return ORIGIN;
    }

    public static Vector2D average(Vector2D[] vs) {

        // TODO

        return ORIGIN;
    }

}