package tech.lindblom.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Constants {

    public class Drive {
        // Left
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 23;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22;

        // Right
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 26;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 25;
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 28;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 27;

        // Encoders
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 60;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 62;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 61;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 59;

        //Drive motor offsets
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.876221;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.8167;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.670;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.116;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.3;

        public static final double DRIVETRAIN_TRACKWIDTH = Units.inchesToMeters(23);
        public static final double DRIVETRAIN_WHEELBASE = Units.inchesToMeters(25);

        public static final double MAX_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
                (Math.hypot(DRIVETRAIN_TRACKWIDTH / 2, DRIVETRAIN_WHEELBASE / 2)));

        public static final double DRIVER_TRANSLATION_RATE_LIMIT = 1.2;
        public static final double DRIVER_ROTATION_RATE_LIMIT = 1.2;
        public static final double DRIVER_ROTATION_INPUT_MULTIPIER = 0.5;

        public static Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(DRIVETRAIN_WHEELBASE / 2,
                DRIVETRAIN_TRACKWIDTH / 2);
        public static Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(DRIVETRAIN_WHEELBASE / 2,
                -DRIVETRAIN_TRACKWIDTH / 2);
        public static Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-DRIVETRAIN_WHEELBASE / 2,
                DRIVETRAIN_TRACKWIDTH / 2);
        public static Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-DRIVETRAIN_WHEELBASE / 2,
                -DRIVETRAIN_TRACKWIDTH / 2);
    }

    public class Auto {
        public static final ShuffleboardTab AUTONOMOUS_TAB = Shuffleboard.getTab("Autonomous");
    }

    public class Vision {
        public static final String FrontCameraName = "Main";

        public static final Transform3d frontCameraPositionOnRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(16), 0, Units.inchesToMeters(-2.5)),
                new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)));


        // These values were also taken from 7525, again, thank you so much.
        public static final double STD_TRUSTABLE_DISTANCE = 6;

        public static final Matrix<N3, N1> SINGLE_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds, if you only see one tag, ie less accuracy/trust so higher values bc we don't trust it
        public static final Matrix<N3, N1> MULTI_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds,  if you see multiple tags, ie more accuracy/trust so lower values bc we trust it

        public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // how significantly important each tag is

    }

    public class Controls {
        public static final double DEADZONE = 0.1;
    }

    public class Climber {
        public static final int LEFT_ARM = 62;
        public static final int RIGHT_ARM = 2;
    }
}
