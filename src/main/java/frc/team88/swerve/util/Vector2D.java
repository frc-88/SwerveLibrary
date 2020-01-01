package frc.team88.swerve.util;

/**
 * Represents a 2D vector with many math utility functions. Immutable. Follows
 * our reference frame convention, which is as follows:
 * 
 * "An angle of 0 degrees is defined as pointing in the direction of the
 * positive y-axis, increasing positively in the counter-clockwise direction and
 * negatively in the clockwise direction. The positive x-axis is at -90
 * degrees."
 */
public class Vector2D {

    // The origin point / zero vector
    public static final Vector2D ORIGIN = createCartesianCoordinates(0, 0);

    // The components of the vector
    private final double x;
    private final double y;

    /**
     * Private constructor using cartesian coordinates.
     * 
     * @param x
     * @param y
     */
    private Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Get the x component of this vector.
     * 
     * @return The x component
     */
    public double getX() {
        return this.x;
    }

    /**
     * Get the y component of this vector.
     * 
     * @return The y component
     */
    public double getY() {
        return this.y;
    }

    /**
     * Get the magnitude of this vector.
     * 
     * @return The magnitude
     */
    public double getMagnitude() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    /**
     * Get the angle of this vector.
     * 
     * @return The angle
     */
    public WrappedAngle getAngle() {
        return new WrappedAngle(Math.toDegrees(Math.atan2(-x, y)));
    }

    /**
     * Creates a Vector2D using the given cartesian coordinates.
     * 
     * @param x
     *              The x component
     * @param y
     *              The y component
     * @return The created vector
     */
    public static Vector2D createCartesianCoordinates(double x, double y) {
        return new Vector2D(x, y);
    }

    /**
     * Creates a vector2D using the given polar coordinates.
     * 
     * @param magnitude
     *                      The magnitude
     * @param angle
     *                      The angle, in degrees
     * @return The created vector
     */
    public static Vector2D createPolarCoordinates(double magnitude, WrappedAngle angle) {
        if (magnitude == 0.) {
            return Vector2D.ORIGIN;
        }
        double angleRad = Math.toRadians(angle.asDouble());
        return createCartesianCoordinates(magnitude * -Math.sin(angleRad), magnitude * Math.cos(angleRad));
    }

    /**
     * Creates a new Vector2D that is a copy of this one, except with the given
     * x component.
     * 
     * @param x The x component for the new vector
     * @return The changed vector
     */
    public Vector2D changeX(double x) {
        return Vector2D.createCartesianCoordinates(x, this.getY());
    }

    /**
     * Creates a new Vector2D that is a copy of this one, except with the given
     * y component.
     * 
     * @param y The y component for the new vector
     * @return The changed vector
     */
    public Vector2D changeY(double y) {
        return Vector2D.createCartesianCoordinates(this.getX(), y);
    }


    /**
     * Creates a new Vector2D that is a copy of this one, except with the given
     * magnitude.
     * 
     * @param x The magnitude for the new vector
     * @return The changed vector
     */
    public Vector2D changeMagnitude(double magnitude) {
        return Vector2D.createPolarCoordinates(magnitude, this.getAngle());
    }

    /**
     * Creates a new Vector2D that is a copy of this one, except with the given
     * angle.
     * 
     * @param angle The angle for the new vector
     * @return The changed vector
     */
    public Vector2D changeAngle(WrappedAngle angle) {
        return Vector2D.createPolarCoordinates(this.getMagnitude(), angle);
    }

    /**
     * Returns this vector added to the given vector.
     * 
     * @param that
     *                 The vector the add to this vector
     * @return The sum
     */
    public Vector2D plus(Vector2D that) {
        return createCartesianCoordinates(this.getX() + that.getX(), this.getY() + that.getY());
    }

    /**
     * Returns this vector multiplied by the given scalar.
     * 
     * @param scalar
     *                   The scalar multiplicand
     * @return The product
     */
    public Vector2D times(double scalar) {
        return this.changeMagnitude(this.getMagnitude() * scalar);
    }

    /**
     * Returns this vector rotated by the given angle
     * 
     * @param angle
     *                  The angle offset
     * @return The rotated vector
     */
    public Vector2D rotate(double angle) {
        return this.changeAngle(this.getAngle().plus(angle));
    }

    /**
     * Determines if this vector is equal to the given vector except for a floating
     * point error
     * 
     * @param that
     *                 The vector to compare this to
     * @return True if the 2 vectors are approximately equal, false otherwise
     */
    public boolean approximatelyEquals(Vector2D that) {
        return MathUtils.doubleEquals(this.getX(), that.getX()) && MathUtils.doubleEquals(this.getY(), that.getY());
    }

    /**
     * Limits the magnitude of the difference between this vector and the returned
     * vector when trying to go from this vector to the desired vector.
     * 
     * @param desired
     *                      The desired vector
     * @param maxChange
     *                      The maximum amount that the returned vector can differ
     *                      from this vector
     * @return The vector that is as close to the desired vector as possible without
     *         exceeing the maximum change
     */
    public Vector2D limitChange(Vector2D desired, double maxChange) {
        // Get the full difference between this vector and the desired vector
        Vector2D difference = desired.plus(this.times(-1));
        // Shorten the magnitude of the difference if it is too large
        if (difference.getMagnitude() > maxChange) {
            difference = difference.changeMagnitude(maxChange);
        }
        // Add the difference back onto this vector
        return this.plus(difference);
    }

}
