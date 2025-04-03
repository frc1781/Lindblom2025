package tech.lindblom.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SPI;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.swerve.DoubleKrakenSwerveModule;
import tech.lindblom.swerve.utils.SwerveSetpoint;
import tech.lindblom.swerve.utils.SwerveSetpointGenerator;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Drive extends Subsystem {
    private final DoubleKrakenSwerveModule frontLeftModule = new DoubleKrakenSwerveModule("Front Left Module",
             Constants.Drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
             Constants.Drive.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_LEFT_MODULE_STEER_ENCODER,
             Constants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET, false);
     private final DoubleKrakenSwerveModule frontRightModule = new DoubleKrakenSwerveModule("Front Right Module",
             Constants.Drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
             Constants.Drive.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_RIGHT_MODULE_STEER_ENCODER,
             Constants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET, true);
     private final DoubleKrakenSwerveModule backLeftModule = new DoubleKrakenSwerveModule("Back Left Module",
             Constants.Drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
             Constants.Drive.BACK_LEFT_MODULE_STEER_MOTOR, Constants.Drive.BACK_LEFT_MODULE_STEER_ENCODER,
             Constants.Drive.BACK_LEFT_MODULE_STEER_OFFSET, false);
     private final DoubleKrakenSwerveModule backRightModule = new DoubleKrakenSwerveModule("Back Right Module",
             Constants.Drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
             Constants.Drive.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER,
             Constants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET, true);

    // Odometry & Kinematics
    public final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(Constants.Drive.FRONT_LEFT_MODULE_POSITION,
            Constants.Drive.FRONT_RIGHT_MODULE_POSITION, Constants.Drive.BACK_LEFT_MODULE_POSITION,
            Constants.Drive.BACK_RIGHT_MODULE_POSITION);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private boolean resetOrientationByDriver;
    SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint currentSetpoint =
            new SwerveSetpoint(
                    new ChassisSpeeds(),
                    new SwerveModuleState[] {
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState()
                    });

    public Drive() {
        super("Drive");
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
        resetOrientationByDriver = false;

        swerveSetpointGenerator =
                new SwerveSetpointGenerator(swerveDriveKinematics, Constants.Drive.MODULE_TRANSLATIONS);
    }

    @Override
    public void init() {
        switch (currentOperatingMode) {
            case AUTONOMOUS:
                break;
        }
    }

    @Override
    public void periodic() {
        updatePoseUsingOdometry();
        Logger.recordOutput(name + "/currentPose", getRobotPose());
        Logger.recordOutput(name + "/swerveModuleStates", getModuleStates());
    }

    public void drive(ChassisSpeeds speeds) {
        // A lot of changes here were made thanks to the code by 6328's open source code, big thanks to them :)
        if (currentOperatingMode == EnumCollection.OperatingMode.DISABLED) {
            System.out.println("MOVING IN DISABLED");
            return;
        }


        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.General.LOOP_PERIOD_SECONDS);
        SwerveModuleState[] setPointStatesUnoptimized = swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint =
                swerveSetpointGenerator.generateSetpoint(
                        Constants.Drive.MODULE_LIMITS,
                        currentSetpoint,
                        discreteSpeeds,
                        Constants.General.LOOP_PERIOD_SECONDS);
        SwerveModuleState[] moduleStates = currentSetpoint.moduleStates();
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND);

        Logger.recordOutput(name + "/requestedSwerveModuleStats", moduleStates);
        Logger.recordOutput(name + "/setPointsUnoptimized", setPointStatesUnoptimized);
        Logger.recordOutput(name + "/requestedSwerveModuleStats", moduleStates);
        Logger.recordOutput(name + "/setPointsUnoptimized", setPointStatesUnoptimized);
        Logger.recordOutput(name + "/ChassisSpeeds", discreteSpeeds);

        //frontLeftModule.runDesiredModuleState(moduleStates[0]);
        //frontRightModule.runDesiredModuleState(moduleStates[1]);
        //backLeftModule.runDesiredModuleState(moduleStates[2]);
        //backRightModule.runDesiredModuleState(moduleStates[3]);
    }

    public void setInitialPose(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(
                getGioRotation(),
                getModulePositions(),
                pose);
    }

    public void updatePoseUsingVisionEstimate(Pose2d estimatedPose, double time, Matrix<N3, N1> stdValue) {
        swerveDrivePoseEstimator.addVisionMeasurement(new Pose2d(estimatedPose.getTranslation(), getRobotRotation()), time, stdValue);
    }

    private void updatePoseUsingOdometry() {
        swerveDrivePoseEstimator.update(getGioRotation(), getModulePositions());
    }

    public Pose2d getRobotPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
        return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getGioRotation() {
        return new Rotation2d(navX.getRotation2d().getRadians());
    }

    public double getGioSpeed() {
        return Math.sqrt(Math.pow(navX.getVelocityX(), 2) + Math.pow(navX.getVelocityY(), 2));  //Pythagorean theorem from navX velocities
    }

    public void orientFieldToRobot() {
        if (resetOrientationByDriver) {  //only do once
            return;
        }
        resetOrientationByDriver = true;
        System.out.println("NOTE: Reoriented concept of zero direction on the field to direction the robot is facing because requested by driver");
        swerveDrivePoseEstimator.resetPosition(
            getGioRotation(), 
            getModulePositions(),
            new Pose2d(getRobotPose().getTranslation(), new Rotation2d()));
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getModulePosition(),
                frontRightModule.getModulePosition(),
                backLeftModule.getModulePosition(),
                backRightModule.getModulePosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeftModule.getCurrentState(),
                frontRightModule.getCurrentState(),
                backLeftModule.getCurrentState(),
                backRightModule.getCurrentState()
        };
    }

   
}
