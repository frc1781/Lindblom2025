package tech.lindblom.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.control.RobotController;

public class EEUtil {
    public static Rotation2d normalizeAngle(Rotation2d angle) {
        double radians = angle.getRadians();

        radians %= 2 * Math.PI;
        if(radians<0) {
            radians += 2 * Math.PI;
        }

        return Rotation2d.fromRadians(radians);
    }

    public static RobotController.SubsystemSetting[] insertElementAtIndex(RobotController.SubsystemSetting[] arr, RobotController.SubsystemSetting newElement, int pos) {
        int i;

        RobotController.SubsystemSetting[] newArray = new RobotController.SubsystemSetting[arr.length + 1];

        for (i = 0; i < arr.length + 1; i++) {
            if (i < pos)
                newArray[i] = arr[i];
            else if (i == pos)
                newArray[i] = newElement;
            else
                newArray[i] = arr[i - 1];
        }
        return newArray;
    }
}
