// Copyrights (c) 2018-2019 FIRST, 2020 Highlanders FRC. All Rights Reserved.

package frc.robot.tools.math;

public class Vector {

    private double i;
    private double j;

    public Vector() {
        i = 0;
        j = 0;
    }

    public Vector(double i, double j) {
        this.i = i;
        this.j = j;
    }

    /**
     * Returns the i-component of the vector.
     *
     * @return The i-component of the vector.
     */
    public double getI() {
        return i;
    }

    /**
     * Returns the j-component of the vector.
     *
     * @return The j-component of the vector.
     */
    public double getJ() {
        return j;
    }

    /**
     * Multiplies the vector by a scalar value.
     *
     * <p>
     * This method modifies the current vector by multiplying each component (i and
     * j) by the given scalar value.
     *
     * @param s The scalar value to multiply the vector by.
     */
    public void scalarMultiple(double s) {
        this.i = this.i * s;
        this.j = this.j * s;
    }

    /**
     * Calculates the dot product of this vector with another vector.
     *
     * <p>
     * The dot product of two vectors is a scalar value that represents the product
     * of their magnitudes and the cosine of the angle
     * between them. It is calculated as the sum of the product of corresponding
     * components of the two vectors.
     *
     * @param u The other vector with which to calculate the dot product.
     * @return The dot product of this vector with the given vector {@code u}.
     */
    public double dot(Vector u) {
        return i * u.getI() + j * u.getJ();
    }

    /**
     * Calculates the magnitude (or length) of the vector.
     *
     * <p>
     * The magnitude of a vector is a non-negative scalar value that represents the
     * length of the vector.
     * It is calculated as the square root of the sum of the squares of its
     * components (i and j).
     *
     * @return The magnitude (or length) of the vector.
     */
    public double magnitude() {
        return Math.sqrt(this.dot(this));
    }

    /**
     * Sets the i-component of the vector.
     *
     * <p>
     * This method modifies the current vector by setting the i-component to the
     * given value.
     *
     * @param i The new value for the i-component of the vector.
     */
    public void setI(double i) {
        this.i = i;
    }

    /**
     * Sets the j-component of the vector.
     *
     * <p>
     * This method modifies the current vector by setting the j-component to the
     * given value.
     *
     * @param j The new value for the j-component of the vector.
     */
    public void setJ(double j) {
        this.j = j;
    }
}