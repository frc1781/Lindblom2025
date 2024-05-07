package tech.lindblom.control;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.Config;
import tech.lindblom.subsystems.DriveSystem;
import tech.lindblom.subsystems.Subsystem;
import tech.lindblom.utils.DriverInput;
import tech.lindblom.utils.DriverInput.ControllerSide;
import tech.lindblom.subsystems.Subsystem.OperatingMode;
import tech.lindblom.subsystems.Subsystem.SubsystemState;
import tech.lindblom.utils.EVector;
import tech.lindblom.utils.Limelight;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Stack;

public class ControlSystem {
    private OperatingMode mCurrentOperatingMode;
    private SubsystemSetting[] mCurrentSettings;;
    private ArrayList<Subsystem> mSubsystems;
    private DriveSystem mDriveSystem;

    // Slew Rate Limiters for controls
    private final SlewRateLimiter mXDriveLimiter = new SlewRateLimiter(Config.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mYDriveLimiter = new SlewRateLimiter(Config.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mRotDriveLimiter = new SlewRateLimiter(Config.DRIVER_ROTATION_RATE_LIMIT);
    private final ProfiledPIDController mLimelightAimController = new ProfiledPIDController(0.070, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mNoteAimController = new ProfiledPIDController(0.035, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mAmpAimController = new ProfiledPIDController(0.035, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mOdometryController = new ProfiledPIDController(4.1, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private boolean mAutoAiming = false;
    private double mAimingAngle = 0.0;
    private double mStrafeDC = 0.0;
    private boolean mCenterOnAprilTagButton = false;
    private boolean mAutoCenterAmp = false;
    private Stack<SubsystemSetting> mSettingStack = new Stack<>();
    private HashMap<Number, Pose2d> aprilTagCoords = new HashMap<>();

    public ControlSystem() {
        mDriveSystem = new DriveSystem();

        mSubsystems = new ArrayList<>();

        mSubsystems.add(mDriveSystem);
        aprilTagCoords.put(1, new Pose2d(15.078597, 0.245597, new Rotation2d()));
        aprilTagCoords.put(2, new Pose2d(16.184259, 0.883391, new Rotation2d()));
        aprilTagCoords.put(3, new Pose2d(16.578467, 4.982443, new Rotation2d()));
        aprilTagCoords.put(4, new Pose2d(16.578467, 5.547593, new Rotation2d()));
        aprilTagCoords.put(5, new Pose2d(14.699883, 8.203925, new Rotation2d()));
        aprilTagCoords.put(6, new Pose2d(1.840625, 8.203925, new Rotation2d()));
        aprilTagCoords.put(7, new Pose2d(-0.038975, 5.547593, new Rotation2d()));
        aprilTagCoords.put(8, new Pose2d(-0.038975, 4.982443, new Rotation2d()));
        aprilTagCoords.put(9, new Pose2d(0.355233, 0.883391, new Rotation2d()));
        aprilTagCoords.put(10, new Pose2d(1.460641, 0.245597, new Rotation2d()));
        aprilTagCoords.put(11, new Pose2d(11.903851, 3.712951, new Rotation2d()));
        aprilTagCoords.put(12, new Pose2d(11.903851, 4.498065, new Rotation2d()));
        aprilTagCoords.put(13, new Pose2d(11.219321, 4.104873, new Rotation2d()));
        aprilTagCoords.put(14, new Pose2d(5.319917, 4.104873, new Rotation2d()));
        aprilTagCoords.put(15, new Pose2d(4.640467, 4.498065, new Rotation2d()));
        aprilTagCoords.put(16, new Pose2d(4.640467, 3.712951, new Rotation2d()));
    }

    public static boolean isRed() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public void manualDriving(EVector driving, EVector rotation, EVector triggers) {
        int redFlip = isRed() ? -1 : 1;

        // forward and backwards
        double xVelocity = -driving.y * redFlip;
        // left and right
        double yVelocity = -driving.x * redFlip;
        // rotation
        double rotVelocity = -rotation.x * Config.DRIVER_ROTATION_INPUT_MULTIPIER + ((triggers.x) - (triggers.y));

        mDriveSystem.driveRaw(
                mAutoCenterAmp ? mStrafeDC
                        : mXDriveLimiter.calculate(xVelocity) * Config.MAX_VELOCITY_METERS_PER_SECOND,
                mYDriveLimiter.calculate(yVelocity) * Config.MAX_VELOCITY_METERS_PER_SECOND,
                mAutoAiming ? mAimingAngle
                        : (mRotDriveLimiter.calculate(rotVelocity) * Config.MAX_VELOCITY_RADIANS_PER_SECOND));
    }


    public void init(OperatingMode operatingMode) {
        for (Subsystem subsystem : mSubsystems) {
            subsystem.setOperatingMode(operatingMode);
        }

        mCurrentOperatingMode = operatingMode;

        switch (operatingMode) {
            case TELEOP:
                mSettingStack.clear();
                mXDriveLimiter.reset(0);
                mYDriveLimiter.reset(0);
                mRotDriveLimiter.reset(0);
                mDriveSystem.setDesiredState(DriveSystem.DriveSystemState.DRIVE_MANUAL);
                break;
            case AUTONOMOUS:
                break;
            default:
                break;
        }
    }

    public void run(DriverInput driverInput) {
        mDriveSystem.genericPeriodic();

        switch (mCurrentOperatingMode) {
            case TELEOP:
                SubsystemState finalDriveState = DriveSystem.DriveSystemState.DRIVE_MANUAL;

                while (!mSettingStack.isEmpty()) {
                    SubsystemSetting setting = mSettingStack.pop();
                    Subsystem subsystem = setting.getSubsystem();
                    SubsystemState state = setting.getState();

                    if (subsystem == mDriveSystem) {
                        finalDriveState = state;
                    }
                }

                mDriveSystem.setDesiredState(finalDriveState);

                EVector driverTriggers = driverInput.getTriggerAxis(Config.DRIVER_CONTROLLER_PORT);
                manualDriving(
                        driverInput.getControllerJoyAxis(ControllerSide.LEFT, Config.DRIVER_CONTROLLER_PORT),
                        driverInput.getControllerJoyAxis(ControllerSide.RIGHT, Config.DRIVER_CONTROLLER_PORT),
                        driverTriggers);
                break;
            case AUTONOMOUS:
                break;
            default:
                break;
        }

        for (Subsystem subsystem : mSubsystems) {
            subsystem.getToState();
        }
    }

    public void disabledPeriodic() {
        for (Subsystem subsystem : mSubsystems) {
            subsystem.disabledPeriodic();
        }
    }

    public void resetNavX() {
        mDriveSystem.resetNavX();
    }
}

class SubsystemSetting {
    public Subsystem mSubsystem;
    public SubsystemState mState;

    public SubsystemSetting(Subsystem subsystem, SubsystemState state) {
        mSubsystem = subsystem;
        mState = state;
    }

    public void setState() {
        mSubsystem.setDesiredState(mState);
    }

    public SubsystemState getState() {
        return mState;
    }

    public Subsystem getSubsystem() {
        return mSubsystem;
    }

    public boolean isFinished() {
        boolean ret_val = mSubsystem.matchesDesiredState();

        return ret_val;
    }

}
