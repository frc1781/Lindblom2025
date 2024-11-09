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
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.swerve.KrakenL2SwerveModule;
import tech.lindblom.swerve.SwerveModule;
import tech.lindblom.utils.Constants;

public class Drive extends StateSubsystem {
    private final SwerveModule mFrontLeft = new KrakenL2SwerveModule("Front Left Module",
            Constants.Drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drive.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_LEFT_MODULE_STEER_ENCODER,
            Constants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mFrontRight = new KrakenL2SwerveModule("Front Right Module",
            Constants.Drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drive.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackLeft = new KrakenL2SwerveModule("Back Left Module",
            Constants.Drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drive.BACK_LEFT_MODULE_STEER_MOTOR, Constants.Drive.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.Drive.BACK_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackRight = new KrakenL2SwerveModule("Back Right Module",
            Constants.Drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drive.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET);

    // Odometry & Kinematics
    public final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(Constants.Drive.FRONT_LEFT_MODULE_POSITION,
            Constants.Drive.FRONT_RIGHT_MODULE_POSITION, Constants.Drive.BACK_LEFT_MODULE_POSITION,
            Constants.Drive.BACK_RIGHT_MODULE_POSITION);

    private final AHRS mNavX = new AHRS(SPI.Port.kMXP);

    private final SwerveDrivePoseEstimator mPoseEstimator;

    public Drive() {
        super("DriveSystem", DriveSystemState.MANUAL);

        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
    }

    @Override
    public void init() {
        switch (currentOperatingMode) {
            case AUTONOMOUS:
                mNavX.reset();
                mNavX.zeroYaw();
                break;
        }
    }

    @Override
    public void getToState() {
        updatePoseUsingOdometry();
    }

    //Use velocities to drive, both the x, y and rotation

    @Override
    public boolean matchesState() {
        return false;
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.runDesiredModuleState(moduleStates[0]);
        mFrontRight.runDesiredModuleState(moduleStates[1]);
        mBackLeft.runDesiredModuleState(moduleStates[2]);
        mBackRight.runDesiredModuleState(moduleStates[3]);
    }

    public void setInitialPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getNavXRotation(), getModulePositions(), pose);
    }

    public void updatePoseUsingVisionEstimate(Pose2d estimatedPose, double time, Matrix<N3, N1> stdValue) {
        mPoseEstimator.addVisionMeasurement(estimatedPose, time, stdValue);
    }

    private void updatePoseUsingOdometry() {
        mPoseEstimator.update(getNavXRotation(), getModulePositions());
        Logger.recordOutput(this.name + "/CurrentPose", getRobotPose());
    }

    public Pose2d getRobotPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
        return mPoseEstimator.getEstimatedPosition().getRotation();
    }

    private Rotation2d getNavXRotation() {
        return new Rotation2d(-mNavX.getRotation2d().getRadians());
    }

    public void zeroNavX() {
        mNavX.setAngleAdjustment(0);
        mNavX.zeroYaw();
        mPoseEstimator.resetPosition(getNavXRotation(), getModulePositions(),
                new Pose2d(getRobotPose().getTranslation(), new Rotation2d()));
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                mFrontLeft.getModulePosition(),
                mFrontRight.getModulePosition(),
                mBackLeft.getModulePosition(),
                mBackRight.getModulePosition()
        };
    }

    public enum DriveSystemState implements SubsystemState {
        MANUAL, AUTO_CONTROLLED
    }
}
