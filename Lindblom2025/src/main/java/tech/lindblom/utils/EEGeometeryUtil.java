package tech.lindblom.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class EEGeometeryUtil {
    public static Rotation2d normalizeAngle(Rotation2d angle) {
        double radians = angle.getRadians();

        radians %= 2 * Math.PI;
        if(radians<0) {
            radians += 2 * Math.PI;
        }

        return Rotation2d.fromRadians(radians);
    }
}
