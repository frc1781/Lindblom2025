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
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0.037842;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0.218506;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0.51123;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0.288818;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.3;
        public static final double DRIVETRAIN_TRACKWIDTH = Units.inchesToMeters(25.75);
        public static final double DRIVETRAIN_WHEELBASE = Units.inchesToMeters(22.75);

        public static final double WHEEL_RADIUS = 0.1016;

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
        public static final String BACK_CAMERA_NAME = "Back";
        public static final String FRONT_LEFT_CAMERA_NAME = "FrontLeft";
        public static final String FRONT_RIGHT_CAMERA_NAME = "BackRight";

        public static final Transform3d FRONT_LEFT_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(1), Units.inchesToMeters(0), Units.inchesToMeters(1)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
        );

        public static final Transform3d FRONT_RIGHT_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(1), Units.inchesToMeters(0), Units.inchesToMeters(1)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
        );

        public static final Transform3d BACK_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(1), Units.inchesToMeters(0), Units.inchesToMeters(1)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
        );


        // These values were also taken from 7525, again, thank you so much.
        public static final double STD_TRUSTABLE_DISTANCE = 6;

        public static final Matrix<N3, N1> SINGLE_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds, if you only see one tag, ie less accuracy/trust so higher values bc we don't trust it
        public static final Matrix<N3, N1> MULTI_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds,  if you see multiple tags, ie more accuracy/trust so lower values bc we trust it

        public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // how significantly important each tag is

    }

    public class Controls {
        public static final double DEADZONE = 0.1;
    }

    public class Elevator {
        public static final int RIGHT_ELEVATOR_MOTOR = 40;
        public static final int LEFT_ELEVATOR_MOTOR = 41;

        
        public static final int FIRST_STAGE_TOF = 0;
        public static final int SECOND_STAGE_TOF = 0;
        public static final int LOWER_TROUGH__TOF = 0;
        // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A5.93175%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A9%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A35%2C%22u%22%3A%22in%22%7D
        public static final double MAX_ELEVATION_MPS = 0.87;

        public static final double ELEVATOR_KS = 0.87; // KS cannot be modeled and needs to be measured
        public static final double ELEVATOR_KG = 0.07;
        public static final double ELEVATOR_KV = 4.60;
        public static final double ELEVATOR_KA = 0.01;

    }
}
