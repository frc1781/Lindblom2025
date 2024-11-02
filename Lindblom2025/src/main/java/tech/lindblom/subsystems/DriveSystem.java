package tech.lindblom.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.swerve.KrakenL2SwerveModule;
import tech.lindblom.swerve.SwerveModule;
import tech.lindblom.utils.Constants;

public class DriveSystem extends Subsystem {
    // Swerve Modules
    private final SwerveModule mFrontLeft = new KrakenL2SwerveModule("Front Left Module",
            Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
            Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mFrontRight = new KrakenL2SwerveModule("Front Right Module",
            Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackLeft = new KrakenL2SwerveModule("Back Left Module",
            Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.BACK_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackRight = new KrakenL2SwerveModule("Back Right Module",
            Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

    // Odometry & Kinematics
    private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(Constants.FRONT_LEFT_MODULE_POSITION,
            Constants.FRONT_RIGHT_MODULE_POSITION, Constants.BACK_LEFT_MODULE_POSITION,
            Constants.BACK_RIGHT_MODULE_POSITION);

    private final SwerveDrivePoseEstimator mPoseEstimator;


    DriveSystem(String name, SubsystemStates defaultState) {
        super(name, defaultState);

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
    public boolean matchesDesiredState() {
        return false;
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                mFrontLeft.getModulePosition(),
                mFrontRight.getModulePosition(),
                mBackLeft.getModulePosition(),
                mBackRight.getModulePosition()
        };
    }
}
