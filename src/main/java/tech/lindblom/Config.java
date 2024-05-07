
package tech.lindblom;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import tech.lindblom.utils.EVector;

public class Config {
    public static final ShuffleboardTab SHUFFLEBOARD_TAB = Shuffleboard.getTab("Electric Eagles");
    public static final ShuffleboardTab AUTONOMOUS_TAB = Shuffleboard.getTab("Autonomous");

    public static final ShuffleboardTab LOG_TAB = Shuffleboard.getTab("Logs");
    public static final ShuffleboardTab CONFIG_TAB = Shuffleboard.getTab("Config");

    public static final EVector RED_SPEAKER_LOCATION = EVector.newVector(16.77, 5.54);
    public static final EVector BLUE_SPEAKER_LOCATION = EVector.newVector(-0.2, 5.54);

    public static final EVector BLUE_PODIUM = EVector.newVector(3,4.7);
    public static final EVector RED_PODIUM = EVector.newVector(15.25,5.55);

    public static final int RED_SPEAKER_APRILTAG = 4;
    public static final int BLUE_SPEAKER_APRILTAG = 7;

    public static final double MAX_SHOOTER_SPEED = 7;
    public static final double MIN_SHOOTER_SPEED = 3.5;

    // CAN IDs
    public static final int PDH_ID = 1;
    public static final int FIRST_PCM_ID = 2;

    // 20-24 Drive train left
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 23;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22;

    // 25-29 Drive train right
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 26;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 25;
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 28;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 27;

    // 40-41 Arm

    public static final int ARM_PIVOT_LEFT_MOTOR = 40;
    public static final int ARM_PIVOT_RIGHT_MOTOR = 41;
    public static final double ARM_POSITION_TOLERANCE = 2.0;
    public static final double ARM_GEAR_RATIO = (1.0/125.0)*(24.0/58.0); // was (1.0/125.0)*(18.0/56.0)
    public static final double ARM_CONVERSION_REL_TO_ANGLE = 73/56.0; //Based on emperical evidence

    // 42-57 Motors
    public static final int COLLECTOR_MOTOR = 57;
    public static final int SHOOTER_TOP_MOTOR = 42;
    public static final int SHOOTER_BOTTOM_MOTOR = 43;
    public static final int LEFT_CLIMBER_MOTOR = 50;
    public static final int RIGHT_CLIMBER_MOTOR = 51;
    public static final int TRAP_HOOK_MOTOR = 52;

    // 60-69 Sensors
    public static final int BOTTOM_SCOLLECTOR_TOF = 58;
    public static final int TOP_SCOLLECTOR_TOF = 56;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 60;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 62;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 61;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 59;

    // Swerve

    public static final double DRIVETRAIN_TRACKWIDTH = Units.inchesToMeters(23);
    public static final double DRIVETRAIN_WHEELBASE = Units.inchesToMeters(25);
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.2;
    public static final double MAX_VELOCITY_FOR_UPDATE = 0.01;
    public static final double MAX_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
            (Math.hypot(DRIVETRAIN_TRACKWIDTH / 2, DRIVETRAIN_WHEELBASE / 2)));
    public static Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(DRIVETRAIN_WHEELBASE / 2,
            DRIVETRAIN_TRACKWIDTH / 2);
    public static Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(DRIVETRAIN_WHEELBASE / 2,
            -DRIVETRAIN_TRACKWIDTH / 2);
    public static Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-DRIVETRAIN_WHEELBASE / 2,
            DRIVETRAIN_TRACKWIDTH / 2);
    public static Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-DRIVETRAIN_WHEELBASE / 2,
            -DRIVETRAIN_TRACKWIDTH / 2);

    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.0454; //
    // -0.919; // -0.075;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.0185;
    // //-0.738; // -0.255;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.8615; //-0.927;
    // // -0.077;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.8688;
    // //-0.272; // -0.735;

    //RALPH's offsets
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.876221; //-0.9755
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.8167;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.670;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.116;
    public static final double ARM_OFFSET = 0.29;

    //RUFUS's offset
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.039;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.115;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.010;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.291;

    // Constants
    public static final double MIN_THRESHOLD_ARM = 0;
    public static final double MAX_THRESHOLD_ARM = 90;

    // First Drivebase

    //Limelight
    public static final String APRILTAG_LIMELIGHT = "limelight-back";
    public static final String NOTE_LIMELIGHT = "limelight-front";
    public static final int NOTE_LIMELIGHT_NOTE_PIPELINE = 0;
    public static final int NOTE_LIMELIGHT_APRILTAG_PIPELINE = 1;

    // Controls
    public static final double JOYSTICK_DEADZONE = 0.1;

    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final double DRIVER_TRANSLATION_RATE_LIMIT = 1.2;
    public static final double DRIVER_ROTATION_RATE_LIMIT = 1.2;
    public static final double DRIVER_ROTATION_INPUT_MULTIPIER = 0.5;

    public static final String RESET_NAVX = "START";
    public static final String DRIVER_REJECT = "BACK";
    public static final String CENTER_AMP = "Y";
    public static final String KEEP_DOWN = "LB";
    public static final String COLLECT = "RB";
    public static final String AUTO_AIM = "A";
    public static final String COLLECT_HIGH = "B";

    // Co-pilot controls
    public static final int CO_PILOT_PORT = 1;
    public static final String TRAP_OUT_BUTTON = "B";

    public static final String NOTE_COLLECTION = "X";
    public static final String SPIT = "LB";
    public static final String SHOOT = "RB";
    public static final String PREPARE_TO_SHOOT = "X";
    public static final String ANGLE_UP = "E";
    public static final String ANGLE_DOWN = "W";
    public static final String SCORE_AMP = "Y";
    public static final String SCORE_PODIUM = "A";
    // public static final String SKIP = "B";
    public static final String LOB = "B";
    public static final String TOGGLE_TRAP = "LT";

    /*
     * ##############################
     * ## SOLENOID CHANNELS
     * ##############################
     */

    public static final int LEFT_HOOK_OPEN = 3;
    public static final int LEFT_HOOK_CLOSE = 10;
    public static final int RIGHT_HOOK_OPEN = 5;
    public static final int RIGHT_HOOK_CLOSE = 8;
    public static final int TRAP_IN = 0;
    public static final int TRAP_OUT = 9;
}