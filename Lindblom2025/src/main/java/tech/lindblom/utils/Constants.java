package tech.lindblom.utils;

import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import tech.lindblom.swerve.utils.ModuleLimits;

public class Constants {

    public class General {
        public static final double LOOP_PERIOD_SECONDS = 0.02;
        public static final boolean DEBUG_MODE = true;
    }

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
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0.38916;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.324463;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0.709473;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0.476074;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.0;
        public static final double INHIBITED_MULTIPLIER = 0.2;
        public static final double DRIVETRAIN_TRACKWIDTH = Units.inchesToMeters(26);
        public static final double DRIVETRAIN_WHEELBASE = Units.inchesToMeters(22.5);

        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

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

        public static final Translation2d[] MODULE_TRANSLATIONS = {
                FRONT_LEFT_MODULE_POSITION,
                FRONT_RIGHT_MODULE_POSITION,
                BACK_LEFT_MODULE_POSITION,
                BACK_RIGHT_MODULE_POSITION
        };

        public static double MAX_LINEAR_SPEED = 4.0;
        public static double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / WHEEL_RADIUS;

        public static final ModuleLimits MODULE_LIMITS =
                new ModuleLimits(MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, Units.degreesToRadians(1080.0));

        public static double TARGET_CORAL_DISTANCE = 0.4;
        public static double ARM_TOF_DISTANCE = 700;
        public static double MAX_TIME_LOOKING_FOR_POLE = 0.5;

        public static int ARM_TOF_ID = 51;
        public static int RIGHT_FRONT_TOF_ID = 52;
        public static int LEFT_FRONT_TOF_ID = 53;

        public static double TARGET_TOF_PARALLEL_DISTANCE = 280;
        public static double TARGET_TOF_PARALLEL_DISTANCE_SHORT = 170;
        public static double TARGET_TOF_CENTERING_PARALLEL_DISTANCE = 250;
        // I have no idea what unit this is in -ally
        public static double TARGET_CORAL_OFFSET_LEFT = -14.2;
        public static double TARGET_CORAL_OFFSET_RIGHT = 2.8;
        public static double TARGET_CORAL_OFFSET_CENTER_CAMERA_1 = -20.5;
    }

    public class Auto {
        public static final ShuffleboardTab AUTONOMOUS_TAB = Shuffleboard.getTab("Autonomous");
    }

    public class Vision {
        public static final String BACK_CAMERA_NAME = "Back";
        public static final String FRONT_LEFT_CAMERA_NAME = "LeftApriltag";
        public static final String FRONT_RIGHT_CAMERA_NAME = "RightApriltag";
        public static final String LEFT_SIDE_CAMERA_NAME = "LeftSide";
        public static final int ERROR_CONSTANT = 1781;

        public static final Transform3d FRONT_LEFT_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(6.75), Units.inchesToMeters(7.75), Units.inchesToMeters(11.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
        );

        public static final Transform3d FRONT_RIGHT_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(-9), Units.inchesToMeters(10.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
        );

        public static final Transform3d BACK_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(4.75), Units.inchesToMeters(14), Units.inchesToMeters(26.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180))
        );

        public static final Transform3d LEFT_SIDE_CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(-15.5), Units.inchesToMeters(0), Units.inchesToMeters(26.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90))
        );


        // These values were also taken from 7525, again, thank you so much.
        public static final double STD_TRUSTABLE_DISTANCE = 6;

        public static final Matrix<N3, N1> SINGLE_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds, if you only see one tag, ie less accuracy/trust so higher values bc we don't trust it
        public static final Matrix<N3, N1> MULTI_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds,  if you see multiple tags, ie more accuracy/trust so lower values bc we trust it

        public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // how significantly important each tag is

    }

    public class Controls {
        public static final double DEADZONE = 0.1;

    }

    public class Arm{
        public static final int ARM_MOTOR_ID = 13;
        public static final int CLAW_CORAL_SENSOR_ID = 54;
    }

    public class Elevator {
        public static final int RIGHT_ELEVATOR_MOTOR = 12;
        public static final int LEFT_ELEVATOR_MOTOR = 11;
        public static final int FIRST_STAGE_TOF = 58;
        public static final int SECOND_STAGE_TOF = 57;
        // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A5.93175%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A9%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A35%2C%22u%22%3A%22in%22%7D
        public static final double MAX_ELEVATION_MPS = 0.87;
        public static final double ELEVATOR_KS = 0.02; // KS cannot be modeled and needs to be measured
        // public static final double ELEVATOR_KG = 0.13;
        // public static final double ELEVATOR_KV = 13.81;
        // Changing these back because may be causing the rope to break
        public static final double ELEVATOR_KG = 0.07;
        public static final double ELEVATOR_KV = 4.60;
        public static final double ELEVATOR_KA = 0.01;
    }

    public class Climber {
        public static final int CLIMBER_MOTOR = 14;
        public static final int SERVO_PWM_PORT = 0;

        public static final double RADIANS_PER_REVOLUTION = (Math.PI * 2) / 125;

        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A120%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A11.1%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A125%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double KS = 0;
        public static final double KG = 3.96;
        public static final double KV = 2.44;
        public static final double KA = 0.14;

        public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
                .p(P)
                .i(I)
                .d(D);
    }

    public class Conveyor {
        public static final int CORAL_HOPPER_SENSOR_FRONT_DIO = 0;
        public static final int CORAL_HOPPER_SENSOR_BACK_DIO = 1;
        public static final int CORAL_CRADLE_SENSOR_DIO = 2;
        public static final int SIDE_RAMP_DIO = 3;
        public static final int BACK_RAMP_DIO = 4;

        //coralConveyor
        public static final int CORAL_CONVEYOR_ID = 15;
    }

    public class Thumb {
        public static final int SPIN_THUMB_MOTOR = 17;
    }
}
