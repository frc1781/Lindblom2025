package tech.lindblom.control;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.routines.TestRoutine;
import tech.lindblom.subsystems.drive.Drive;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RobotController {
    Drive driveSystem;
    Vision visionSystem;
    Auto autoSystem;
    LEDs ledsSystem;

    DriverInput driverInput;

    private ArrayList<StateSubsystem> stateSubsystems;
    private ArrayList<Subsystem> subsystems;

    private final SlewRateLimiter xControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter yControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter rotControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_ROTATION_RATE_LIMIT);

    Pose2d empty = new Pose2d();

    public RobotController() {
        driveSystem = new Drive();
        autoSystem = new Auto(
                new TestRoutine()
        );
        visionSystem = new Vision();
        ledsSystem = new LEDs();

        stateSubsystems = new ArrayList<>();

        driverInput = new DriverInput(this);

        stateSubsystems.add(driveSystem);
        stateSubsystems.add(ledsSystem);

        subsystems = new ArrayList<>();
        subsystems.add(visionSystem);
        subsystems.add(autoSystem);
    }

    public enum Action {
        WAIT
    }

    public void init(EnumCollection.OperatingMode mode) {
        switch (mode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                    setInitialRobotPose(mode);
                break;
            case TELEOP:
                    setInitialRobotPose(mode);
                break;
            case TEST:
                break;
            case SIMULATION:
                break;
            default:
                System.out.println("WHAT IS HAPPENING");
                break;
        }

        for (Subsystem subsystem : subsystems) {
            subsystem.setOperatingMode(mode);
            subsystem.init();
        }

        for (StateSubsystem subsystem : stateSubsystems) {
            subsystem.setOperatingMode(mode);
            subsystem.init();
        }
    }

    public void run(EnumCollection.OperatingMode mode) {
        switch (mode) {
            case DISABLED:
                visionUpdates();
                break;
            case AUTONOMOUS:
                visionUpdates();
                processAutonomousInputs();
                break;
            case TELEOP:
                visionUpdates();
                processDriverInputs();
                break;
            case TEST:
                break;
            case SIMULATION:
                break;
            default:
                System.out.println("WHAT IS HAPPENING");
                break;
        }

        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }

        for (StateSubsystem subsystem : stateSubsystems) {
            subsystem.getToState();
        }
    }

    public void processAutonomousInputs() {

    }

    public void processDriverInputs() {
        DriverInput.InputHolder holder = driverInput.getDriverInputs();
        driverDriving(holder.driverLeftJoystickPosition, holder.driverRightJoystickPosition);
        List<StateSubsystem> setSubsystems = new ArrayList<>();

        if (holder.resetNavX) {
            driveSystem.zeroNavX();
        }

        for (int i = 0; i < holder.requestedSubsystemSettings.size(); i++) {
            SubsystemSetting setting = holder.requestedSubsystemSettings.get(i);
            System.out.println(setting.subsystem + " " + setting.state);
            setting.subsystem.setState(setting.state);
            setSubsystems.add(setting.subsystem);
        }

        for (StateSubsystem subsystem : stateSubsystems) {
            if (subsystem.getCurrentState() != subsystem.defaultState && !setSubsystems.contains(subsystem)) {
                subsystem.setDefaultState();
            }
        }
    }

    public void driverDriving(Translation2d translation, Translation2d rotation) {
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        int flipForRed = isRed ? -1 : 1;

        double xVelocity = -translation.getY() * flipForRed;
        double yVelocity = -translation.getX() * flipForRed;
        double rotVelocity = -rotation.getX() * Constants.Drive.DRIVER_ROTATION_INPUT_MULTIPIER;

        Logger.recordOutput("Driver/movement/x", translation.getX());
        Logger.recordOutput("Driver/rotation/x", rotation.getX());

        Logger.recordOutput("Driver/movement/y", translation.getY());
        Logger.recordOutput("Driver/rotation/y", rotation.getY());

        double xSpeed = xControllerLimiter.calculate(xVelocity) * Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND;
        double ySpeed = yControllerLimiter.calculate(yVelocity) * Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND;
        double rotSpeed = rotControllerLimiter.calculate(rotVelocity) * Constants.Drive.MAX_VELOCITY_RADIANS_PER_SECOND;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                xSpeed,
                ySpeed,
                rotSpeed), driveSystem.getRobotRotation());
        driveSystem.drive(speeds);
    }

    public void visionUpdates() {
        Optional<Pose2d> visionEstimateOptional = visionSystem.getFrontCameraPose();
        if (visionEstimateOptional.isPresent()) {
            Pose2d visionEstimate = visionEstimateOptional.get();
            Logger.recordOutput("RobotController/updatingUsingVision", true);

            driveSystem.updatePoseUsingVisionEstimate(
                    visionEstimate,
                    Timer.getFPGATimestamp(),
                    visionSystem.getEstimationStdDevs(
                            visionEstimate,
                            visionSystem.getFrontCameraPipelineResult()
                    )
            );
        }

        Logger.recordOutput("RobotController/updatingUsingVision", false);
    }

    public void setInitialRobotPose(EnumCollection.OperatingMode mode) {
        Optional<Pose2d> visionPoseOptional = visionSystem.getFrontCameraPose();
        PhotonPipelineResult pipelineResult = visionSystem.getFrontCameraPipelineResult();

        if (visionPoseOptional.isPresent() && pipelineResult != null) {
            Pose2d visionPose = visionPoseOptional.get();

            if (pipelineResult.targets.size() > 1) {
                driveSystem.setInitialPose(visionPose);
                return;
            }
        }

        if (mode == EnumCollection.OperatingMode.AUTONOMOUS) {
            try {
                Pose2d poseFromPath = autoSystem.getStartPosition();
                driveSystem.setInitialPose(poseFromPath);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (mode == EnumCollection.OperatingMode.TELEOP) {
            driveSystem.setInitialPose(empty);
        }
    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    static class SubsystemSetting {
        public StateSubsystem subsystem;
        public StateSubsystem.SubsystemState state;
        public int weight;

        public SubsystemSetting(StateSubsystem subsystem, StateSubsystem.SubsystemState state, int weight) {
            this.subsystem = subsystem;
            this.state = state;
            this.weight = weight;
        }
    }
}
