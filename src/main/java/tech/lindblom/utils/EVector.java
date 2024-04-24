package tech.lindblom.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Very similar to the PVector class, except it has an E in it.
 * 
 * @author Vincent Dizon
 */

public class EVector {
    public double x, y, z;

    /**
     * 
     * @param inputs 0-3 inputs
     */
    public EVector(double... inputs) {
        switch (inputs.length) {
            case 1:
                x = inputs[0];
                y = 0;
                z = 0;
                break;
            case 2:
                x = inputs[0];
                y = inputs[1];
                z = 0;
                break;
            case 3:
                x = inputs[0];
                y = inputs[1];
                z = inputs[2];
                break;
        }
    }

    public static EVector fromPose(Pose2d pose) {
        return new EVector(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    public static EVector newVector(double... inputs) {
        return new EVector(inputs);
    }

    public static EVector fromPose2d(Pose2d input) {
        return new EVector(input.getTranslation().getX(), input.getTranslation().getY(),
                input.getRotation().getRadians());
    }

    public double dist(EVector other) {
        return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2) + Math.pow(other.z - z, 2));
    }

    public void set(double... inputs) {
        switch (inputs.length) {
            case 1:
                x = inputs[0];
                y = 0;
                z = 0;
                break;
            case 2:
                x = inputs[0];
                y = inputs[1];
                z = 0;
                break;
            case 3:
                x = inputs[0];
                y = inputs[1];
                z = inputs[2];
                break;
        }
    }

    public static EVector fromAngle(double angle, EVector vector) {
        return new EVector(vector.x * Math.cos(angle), vector.y * Math.sin(angle), vector.z);
    }

    public static EVector fromAngle(double angle) {
        return new EVector(Math.cos(angle), Math.sin(angle), 0);
    }

    public static EVector positionWithDegrees(double x, double y, double degrees) {
        return new EVector(x, y, Math.toRadians(degrees));
    }

    public Pose2d toPose2d() {
        return new Pose2d(x,y, new Rotation2d(z));
    }

    public EVector rotate2d(double angle) {
        return new EVector(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle), z);
    }

    public double magnitude(){
        return Math.sqrt(
            Math.pow(x, 2) + Math.pow(y,2)
        );
    }

    public double heading() {
        return Math.atan2(y, x);
    }

    public double angleBetween(EVector other) {
        EVector deltaVector = other.sub(this);
        return Math.atan(deltaVector.y / deltaVector.x);
    }

    public EVector copy() {
        return new EVector(x, y, z);
    }

    public EVector withX(double x) {
        EVector ret_val = this.copy();
        ret_val.x = x;

        return ret_val;
    }

    public EVector withY(double y) {
        EVector ret_val = this.copy();
        ret_val.y = y;

        return ret_val;
    }

    public EVector withZ(double z) {
        EVector ret_val = this.copy();
        ret_val.z = z;

        return ret_val;
    }

    public EVector withZDegrees(double z) {
        return this.withZ(Math.toRadians(z));
    }

    public EVector add(EVector vector) {
        return new EVector(x + vector.x, y + vector.y, z + vector.z);
    }

    public EVector sub(EVector vector) {
        return new EVector(x - vector.x, y - vector.y, z - vector.z);
    }

    public EVector mult(double scalar) {
        return new EVector(x * scalar, y * scalar, z * scalar);
    }

    public EVector div(double scalar) {
        return new EVector(x / scalar, y / scalar, z / scalar);
    }

    public EVector rotate(double angle) {
        return new EVector(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle), z);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }
}
