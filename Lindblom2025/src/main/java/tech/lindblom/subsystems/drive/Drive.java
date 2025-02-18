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

import java.lang.ModuleLayer.Controller;
import java.util.ResourceBundle.Control;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.swerve.DoubleKrakenSwerveModule;
import tech.lindblom.swerve.KrakenL2SwerveModule;
import tech.lindblom.swerve.SwerveModule;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Drive extends Subsystem {
/*    private final SwerveModule frontLeftModule = new DoubleKrakenSwerveModule("Front Left Module",
             Constants.Drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
             Constants.Drive.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_LEFT_MODULE_STEER_ENCODER,
             Constants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET, false);
     private final SwerveModule frontRightModule = new DoubleKrakenSwerveModule("Front Right Module",
             Constants.Drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
             Constants.Drive.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_RIGHT_MODULE_STEER_ENCODER,
             Constants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET, false);
     private final SwerveModule backLeftModule = new DoubleKrakenSwerveModule("Back Left Module",
             Constants.Drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
             Constants.Drive.BACK_LEFT_MODULE_STEER_MOTOR, Constants.Drive.BACK_LEFT_MODULE_STEER_ENCODER,
             Constants.Drive.BACK_LEFT_MODULE_STEER_OFFSET, true);
     private final SwerveModule backRightModule = new DoubleKrakenSwerveModule("Back Right Module",
             Constants.Drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
             Constants.Drive.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER,
             Constants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET, false);*/
    private final SwerveModule backRightModule = new DoubleKrakenSwerveModule("Front Left Module",
            Constants.Drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drive.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_LEFT_MODULE_STEER_ENCODER,
            Constants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET, false);
    private final SwerveModule backLeftModule = new DoubleKrakenSwerveModule("Front Right Module",
            Constants.Drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drive.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET, false);
    private final SwerveModule frontRightModule = new DoubleKrakenSwerveModule("Back Left Module",
            Constants.Drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drive.BACK_LEFT_MODULE_STEER_MOTOR, Constants.Drive.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.Drive.BACK_LEFT_MODULE_STEER_OFFSET, true);
    private final SwerveModule frontLeftModule = new DoubleKrakenSwerveModule("Back Right Module",
            Constants.Drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drive.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET, false);
    // Odometry & Kinematics
    public final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(Constants.Drive.FRONT_LEFT_MODULE_POSITION,
            Constants.Drive.FRONT_RIGHT_MODULE_POSITION, Constants.Drive.BACK_LEFT_MODULE_POSITION,
            Constants.Drive.BACK_RIGHT_MODULE_POSITION);

/*    public final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(Constants.Drive.BACK_RIGHT_MODULE_POSITION,
            Constants.Drive.BACK_LEFT_MODULE_POSITION, Constants.Drive.FRONT_RIGHT_MODULE_POSITION,
            Constants.Drive.FRONT_LEFT_MODULE_POSITION);*/

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public Drive() {
        super("Drive");
        navX.resetDisplacement();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
    }

    @Override
    public void init() {
        switch (currentMode) {
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
        if (currentMode == EnumCollection.OperatingMode.DISABLED) {
            System.out.println("MOVING IN DISABLED");
            return;
        }

        SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND);

        Logger.recordOutput(name + "/requestedSwerveModuleStats", moduleStates);

        frontLeftModule.runDesiredModuleState(moduleStates[0]);
        frontRightModule.runDesiredModuleState(moduleStates[1]);
        backLeftModule.runDesiredModuleState(moduleStates[2]);
        backRightModule.runDesiredModuleState(moduleStates[3]);
    }
    
    public void driveSide(boolean left, ChassisSpeeds speeds) {
        SwerveModuleState zeroSpeedModuleStates = new SwerveModuleState(0, 0);
        SwerveModuleState speedModuleStates = new SwerveModuleState(speeds, 0); 

        if (left == true) {
            frontLeftModule.runDesiredModuleState(zeroSpeedModuleState[0]);
            frontRightModule.runDesiredModuleState(speedModuleState[0]);
            backLeftModule.runDesiredModuleState(zeroSpeedModuleState[1]);
            backRightModule.runDesiredModuleState(speedModuleState[1]);
        }
        if (left == false) {
            frontLeftModule.runDesiredModuleState(speedModuleState[0]);
            frontRightModule.runDesiredModuleState(zeroSpeedModuleState);
            backLeftModule.runDesiredModuleState(speedModuleState[1]);
            backRightModule.runDesiredModuleState(zeroSpeedModuleState);
        }
    }

    public void setInitialPose(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
    }

    public void updatePoseUsingVisionEstimate(Pose2d estimatedPose, double time, Matrix<N3, N1> stdValue) {
        swerveDrivePoseEstimator.addVisionMeasurement(estimatedPose, time, stdValue);
    }

    private void updatePoseUsingOdometry() {
        swerveDrivePoseEstimator.update(getRotation(), getModulePositions());
    }

    public Pose2d getRobotPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
        return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getRotation() {
        return new Rotation2d(navX.getRotation2d().getRadians());
    }
    
    public void resetNavX() {
        navX.reset();
    }

    public void zeroRotation() {
        navX.setAngleAdjustment(0);
        navX.zeroYaw();
        
        swerveDrivePoseEstimator.resetPosition(getRotation(), getModulePositions(),
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
