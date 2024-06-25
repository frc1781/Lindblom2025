package tech.lindblom.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.Config;
import tech.lindblom.swerve.KrakenL2SwerveModule;
import tech.lindblom.swerve.SwerveModule;

public class DriveSystem extends Subsystem {

    // Swerve Modules
    private final SwerveModule mFrontLeft = new KrakenL2SwerveModule("Front Left Module",
            Config.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Config.FRONT_LEFT_MODULE_STEER_MOTOR, Config.FRONT_LEFT_MODULE_STEER_ENCODER,
            Config.FRONT_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mFrontRight = new KrakenL2SwerveModule("Front Right Module",
            Config.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Config.FRONT_RIGHT_MODULE_STEER_MOTOR, Config.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Config.FRONT_RIGHT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackLeft = new KrakenL2SwerveModule("Back Left Module",
            Config.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Config.BACK_LEFT_MODULE_STEER_MOTOR, Config.BACK_LEFT_MODULE_STEER_ENCODER,
            Config.BACK_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackRight = new KrakenL2SwerveModule("Back Right Module",
            Config.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Config.BACK_RIGHT_MODULE_STEER_MOTOR, Config.BACK_RIGHT_MODULE_STEER_ENCODER,
            Config.BACK_RIGHT_MODULE_STEER_OFFSET);

    // Odometry & Kinematics
    private SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(Config.FRONT_LEFT_MODULE_POSITION,
            Config.FRONT_RIGHT_MODULE_POSITION, Config.BACK_LEFT_MODULE_POSITION,
            Config.BACK_RIGHT_MODULE_POSITION);

    private SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(mKinematics, getRobotAngle(), getModulePositions(), getRobotPose());

    private SwerveDrivePoseEstimator mPoseEstimator;
    private boolean mIsFieldOriented = true;
    private double mNavXOffset = 0;
    // Sensors
    private AHRS mNavX = new AHRS(SPI.Port.kMXP);

    public DriveSystem() {
        super("Drive System", DriveSystemState.DRIVE_MANUAL);
        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
        mNavX.resetDisplacement();

        AutoBuilder.configureHolonomic(
            this::getRobotPose, // Robot pose supplier
            this::resetRobotPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveWithChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public enum DriveSystemState implements Subsystem.SubsystemState {
        DRIVE_MANUAL,
        SYSID,
    }

    @Override
    public void getToState() {
        // System.out.println(getState());
        switch ((DriveSystemState) getState()) {
            case DRIVE_MANUAL:
                if (super.currentMode == OperatingMode.AUTONOMOUS) {
                    // driveRaw(0, 0, 0);
                }
                break;
            case SYSID:
                driveRaw(1, 0, 0);
                ChassisSpeeds currentSpeeds = getChassisSpeeds();
                System.out.print(super.currentTime + "," + currentSpeeds.vxMetersPerSecond + ","
                        + currentSpeeds.vyMetersPerSecond + "," + currentSpeeds.omegaRadiansPerSecond);
                break;
            default:
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((DriveSystemState) getState()) {
            case DRIVE_MANUAL:
                return true;
            default:
                return false;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void genericPeriodic() {
        updateOdometry();
    }

    @Override
    public void init() {
        mFrontLeft.init();
        mFrontRight.init();
        mBackLeft.init();
        mBackRight.init();

        switch (currentMode) {
            case TELEOP:
                mIsFieldOriented = false;

                setOdometry(Config.RED_PODIUM.toPose2d());
                setDesiredState(DriveSystemState.DRIVE_MANUAL);
                break;
            default:
                break;
        }
    }

    public void setOdometry(Pose2d pose) {
        mOdometry.resetPosition(getRobotAngle(), getModulePositions(), pose);
    }

    public void driveRaw(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(
                mIsFieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                                xSpeed,
                                ySpeed,
                                rot),
                        getRobotAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Config.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);

        Logger.recordOutput("DriveSystem/ModuleStates", moduleStates);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        if (getState() == DriveSystemState.DRIVE_MANUAL)
            return;
        // System.out.println("Driving: " + speeds.vxMetersPerSecond + " " +
        // speeds.vyMetersPerSecond + " "
        // + speeds.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 4.2);

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);
    }

    public Rotation2d getRobotAngle() {
        double reportedVal = -mNavX.getRotation2d().getRadians();

        reportedVal %= 2 * Math.PI;
        if (reportedVal < 0) {
            reportedVal += 2 * Math.PI;
        }

        return new Rotation2d(reportedVal);
    }

    public Pose2d getRobotPose() {
        return new Pose2d(mPoseEstimator.getEstimatedPosition().getTranslation(), getRobotAngle());
    }

    public void resetRobotPose(Pose2d pose2d) {
        mPoseEstimator.resetPosition(getRobotAngle(), getModulePositions(), pose2d);
    }


    public ChassisSpeeds getChassisSpeeds() {
        return mKinematics.toChassisSpeeds(getModuleStates());
    }

    private void updateOdometry() {
        Pose2d currentOdometry = mPoseEstimator.update(getRobotAngle(), getModulePositions());
        Logger.recordOutput("DriveSystem/Odometry", currentOdometry);
    }

    public void resetNavX() {
        mNavX.reset();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                mFrontLeft.getModulePosition(),
                mFrontRight.getModulePosition(),
                mBackLeft.getModulePosition(),
                mBackRight.getModulePosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                mFrontLeft.getCurrentState(),
                mFrontRight.getCurrentState(),
                mBackLeft.getCurrentState(),
                mFrontLeft.getCurrentState()
        };
    }
}
