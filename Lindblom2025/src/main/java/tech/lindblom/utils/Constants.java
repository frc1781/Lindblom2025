package tech.lindblom.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
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
        public static Slot0Configs steerGains = new Slot0Configs()
                .withKP(1).withKI(0).withKD(0.01)
                .withKS(0.1).withKV(0).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        public static Slot0Configs driveGains = new Slot0Configs()
                .withKP(0).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);

        public static SwerveModuleConstants.ClosedLoopOutputType steerClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
        public static SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

        public static SwerveModuleConstants.DriveMotorArrangement driveMotorType = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
        public static SwerveModuleConstants.SteerMotorArrangement steerMotorType = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;
        public static SwerveModuleConstants.SteerFeedbackType steerFeedbackType = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder;
        public static double slipCurrent = 120;

        public static TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()

                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(1 / (0.1016 * Math.PI * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * .9766))
                );

        public static TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withSupplyCurrentLimitEnable(true)
                );

        public static CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        public static final double DRIVER_TRANSLATION_RATE_LIMIT = 1.2;
        public static final double DRIVER_ROTATION_RATE_LIMIT = 1.2;
        public static final double DRIVER_ROTATION_INPUT_MULTIPIER = 0.5;

        public static double speedAt12Volts = 4.2;

        public static double coupleRatio = (double) 150 / 7;
        public static double driveGearRatio = 6.75;
        public static double steerGearRatio = (double) 150 / 7;

        public static double wheelRadius = Units.inchesToMeters(4); //Inches

        public static boolean invertLeftSide = true;
        public static boolean invertRightSide = false;

        public static double steerInertia = 0.01;
        public static double driveInertia = 0.01;

        public static double steerFrictionVoltage = 0.2;
        public static double driveFrictionVoltage = 0.2;

        public static SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(driveGearRatio)
                .withSteerMotorGearRatio(steerGearRatio)
                .withCouplingGearRatio(coupleRatio)
                .withWheelRadius(wheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSlipCurrent(slipCurrent)
                .withSpeedAt12Volts(speedAt12Volts)
                .withDriveMotorType(driveMotorType)
                .withSteerMotorType(steerMotorType)
                .withFeedbackSource(steerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withSteerInertia(steerInertia)
                .withDriveInertia(driveInertia)
                .withSteerFrictionVoltage(steerFrictionVoltage)
                .withDriveFrictionVoltage(driveFrictionVoltage);

        // Front Left
        public static int frontLeftDriveMotorId = 21;
        public static int frontLeftSteerMotorId = 20;
        public static int frontLeftEncoderId = 60;
        public static double frontLeftEncoderOffset = 0.454345703125;
        public static boolean frontLeftSteerMotorInverted = false;
        public static boolean frontLeftEncoderInverted = true;

        public static double frontLeftXPos = Units.inchesToMeters(11.375);
        public static double frontLeftYPos = Units.inchesToMeters(11.375);

        // Front Right
        public static int frontRightDriveMotorId = 26;
        public static int frontRightSteerMotorId = 25;
        public static int frontRightEncoderId = 62;
        public static double frontRightEncoderOffset = -0.21728515625;
        public static boolean frontRightSteerMotorInverted = false;
        public static boolean frontRightEncoderInverted = true;

        public static double frontRightXPos = Units.inchesToMeters(11.375);
        public static double frontRightYPos = -Units.inchesToMeters(-11.375);

        // Back Left
        public static int backLeftDriveMotorId = 23;
        public static int backLeftSteerMotorId = 22;
        public static int backLeftEncoderId = 61;
        public static double backLeftEncoderOffset = 0.484619140625;
        public static boolean backLeftSteerMotorInverted = false;
        public static boolean backLeftEncoderInverted = true;

        public static double backLeftXPos = Units.inchesToMeters(-11.375);
        public static double backLeftYPos = Units.inchesToMeters(11.375);

        // Back Right
        public static int backRightDriveMotorId = 28;
        public static int backRightSteerMotorId = 27;
        public static int backRightEncoderId = 59;
        public static double backRightEncoderOffset = -0.286865234375;
        public static boolean backRightSteerMotorInverted = false;
        public static boolean backRightEncoderInverted = true;

        public static double backRightXPos = Units.inchesToMeters(-11.375);
        public static double backRightYPos = Units.inchesToMeters(-11.375);

        public static SwerveModuleConstants FrontLeft = ConstantCreator
                .createModuleConstants
                        (frontLeftSteerMotorId,
                                frontLeftDriveMotorId,
                                frontLeftEncoderId,
                                frontLeftEncoderOffset,
                                frontLeftXPos,
                                frontLeftYPos,
                                invertLeftSide,
                                frontLeftSteerMotorInverted,
                                frontLeftEncoderInverted
                        );

        public static SwerveModuleConstants FrontRight = ConstantCreator
                .createModuleConstants
                        (frontRightSteerMotorId,
                                frontRightDriveMotorId,
                                frontRightEncoderId,
                                frontRightEncoderOffset,
                                frontRightXPos,
                                frontRightYPos,
                                invertRightSide,
                                frontRightSteerMotorInverted,
                                frontRightEncoderInverted
                        );

        public static SwerveModuleConstants BackLeft = ConstantCreator
                .createModuleConstants
                        (backLeftSteerMotorId,
                                backLeftDriveMotorId,
                                backLeftEncoderId,
                                backLeftEncoderOffset,
                                backLeftXPos,
                                backLeftYPos,
                                invertLeftSide,
                                backLeftSteerMotorInverted,
                                backLeftEncoderInverted
                        );

        public static SwerveModuleConstants BackRight = ConstantCreator
                .createModuleConstants
                        (backRightSteerMotorId,
                                backRightDriveMotorId,
                                backRightEncoderId,
                                backRightEncoderOffset,
                                backRightXPos,
                                backRightYPos,
                                invertRightSide,
                                backRightSteerMotorInverted,
                                backRightEncoderInverted
                        );
    }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
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
}
