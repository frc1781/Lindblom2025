package tech.lindblom.control;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.routines.TestRoutine;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

// The robot controller, controls robot.
public class RobotController {
    public DriveController driveController;
    public Vision visionSystem;
    public Auto autoSystem;
    public LEDs ledsSystem;

    DriverInput driverInput;

    public Timer autoTimer = new Timer();

    private Action currentAction;
    private ArrayList<StateSubsystem> stateSubsystems;
    private ArrayList<Subsystem> subsystems;

    private final SlewRateLimiter xControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter yControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter rotControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_ROTATION_RATE_LIMIT);

    private HashMap<Action, SubsystemSetting[]> actionMap = new HashMap<>();

    public RobotController() {
        driveController = new DriveController(this);
        autoSystem = new Auto(this,
                new TestRoutine()
        );
        visionSystem = new Vision();
        ledsSystem = new LEDs();
        driverInput = new DriverInput(this);

        stateSubsystems = new ArrayList<>();
        stateSubsystems.add(ledsSystem);

        subsystems = new ArrayList<>();
        subsystems.add(driveController);
        subsystems.add(visionSystem);
        subsystems.add(autoSystem);
        createActions();
    }

    public enum Action {
        WAIT,
        TEST_RED,
        TEST_BLUE,
        TEST_GREEN
    }

    public void init(EnumCollection.OperatingMode mode) {
        switch (mode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                break;
            case TELEOP:
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
                if (currentAction != null) {
                    SubsystemSetting[] subsystemSettings = actionMap.get(currentAction);
                    for (SubsystemSetting subsystemSetting : subsystemSettings) {
                        if (subsystemSetting.subsystem.getCurrentState() == subsystemSetting.state) continue;

                        subsystemSetting.subsystem.setState(subsystemSetting.state);
                    }
                }
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
            subsystem.periodic();
        }
    }

    public void processDriverInputs() {
        DriverInput.InputHolder holder = driverInput.getDriverInputs();
        driverDriving(holder.driverLeftJoystickPosition, holder.driverRightJoystickPosition);
        List<StateSubsystem> setSubsystems = new ArrayList<>();

        if (holder.resetNavX) {
            driveController.resetNavX();
        }

        for (int i = 0; i < holder.requestedSubsystemSettings.size(); i++) {
            SubsystemSetting setting = holder.requestedSubsystemSettings.get(i);
            setting.subsystem.setState(setting.state);
            setSubsystems.add(setting.subsystem);
        }

        for (StateSubsystem subsystem : stateSubsystems) {
            if (subsystem.getCurrentState() != subsystem.defaultState && !setSubsystems.contains(subsystem)) {
                subsystem.restoreToDefaultState();
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

        driveController.driveUsingVelocities(xSpeed, ySpeed, rotSpeed);
    }

    public void visionUpdates() {
        Optional<Pose2d> visionEstimateOptional = visionSystem.getFrontCameraPose();
        if (visionEstimateOptional.isPresent()) {
            Pose2d visionEstimate = visionEstimateOptional.get();
            PhotonPipelineResult pipelineResult = visionSystem.getFrontCameraPipelineResult();
            Logger.recordOutput("RobotController/updatingUsingVision", true);
            if (pipelineResult.targets.size() > 1) {
                driveController.updatePoseUsingVisionEstimate(
                        visionEstimate,
                        Timer.getFPGATimestamp(),
                        visionSystem.getEstimationStdDevs(
                                visionEstimate,
                                pipelineResult
                        )
                );
            }
        }

        Logger.recordOutput("RobotController/updatingUsingVision", false);
    }

    public void setAutoStep(AutoStep autoStep) {
        if (autoStep == null) return;

        switch (autoStep.getStepType()) {
            case ACTION:
                setAction(autoStep.getAction());
                break;
            case PATH:
                driveController.setAutoPath(autoStep.getPath());
                break;
            case PATH_AND_ACTION:
                driveController.setAutoPath(autoStep.getPath());
                setAction(autoStep.getAction());
                break;
        }
    }

    public void setAction(Action action) {
        currentAction = action;
    }

    public void interruptAction() {
        currentAction = null;
        driveController.setAutoPath(null);

        for (int i = 0; i < stateSubsystems.size(); i++) {
            stateSubsystems.get(i).restoreToDefaultState();
        }
    }

    public void createActions() {
        defineAction(Action.TEST_BLUE,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.BLUE, 0));
        defineAction(Action.TEST_RED,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.RED, 0));
        defineAction(Action.TEST_GREEN,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.GREEN, 0));
    }

    public void defineAction(Action action, SubsystemSetting... settings) {
        actionMap.put(action, settings);
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
