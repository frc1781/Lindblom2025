package tech.lindblom.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(Constants.Drive.FRONT_LEFT_MODULE_POSITION,
            Constants.Drive.FRONT_RIGHT_MODULE_POSITION, Constants.Drive.BACK_LEFT_MODULE_POSITION,
            Constants.Drive.BACK_RIGHT_MODULE_POSITION);

    private final SwerveDrivePoseEstimator mPoseEstimator;

    public Drive() {
        super("DriveSystem", DriveSystemState.MANUAL);

        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
    }

    @Override
    public void init() {

    }

    @Override
    public void getToState() {

    }

    @Override
    public boolean matchesState() {
        return false;
    }

    public void updatePoseUsingVisionEstimate() {

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
