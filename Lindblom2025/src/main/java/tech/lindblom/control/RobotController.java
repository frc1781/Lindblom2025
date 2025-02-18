package tech.lindblom.control;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import tech.lindblom.subsystems.arm.Arm;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.routines.*;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.elevator.Elevator;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.NoSuchElementException;

// The robot controller, controls robot.
public class RobotController {
    public DriveController driveController;
    public Vision visionSystem;
    public Auto autoSystem;
    public LEDs ledsSystem;
    public Elevator elevatorSystem;
    public Arm armSystem;
    //public BaseClimber climberSystem;

    DriverInput driverInput;

    public Timer autoTimer = new Timer();

    private Action currentAction;
    private ArrayList<StateSubsystem> stateSubsystems;
    private ArrayList<Subsystem> subsystems;

    private final SlewRateLimiter xControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter yControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter rotControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_ROTATION_RATE_LIMIT);

    private final HashMap<Action, SubsystemSetting[]> actionMap = new HashMap<>();

    private DriverInput.InputHolder mostRecentInputHolder;
    private ArrayList<SequentialActionStatus> sequentialActionStatus = new ArrayList<>();
    private Action currentSequentialAction = null;
    private EnumCollection.OperatingMode currentOperatingMode = null;

    public RobotController() {
        driveController = new DriveController(this);
        autoSystem = new Auto(this,
                new TestRoutine(),
                new Collect(),
                new OneCoralAuto()
        );
        visionSystem = new Vision(this);
        ledsSystem = new LEDs();
        elevatorSystem = new Elevator();
        armSystem = new Arm(this);
        driverInput = new DriverInput(this);
/*        if (RobotBase.isReal()) {*/
            //climberSystem = new Climber();
/*        } else {
            climberSystem = new ClimberSim();
        }*/
        stateSubsystems = new ArrayList<>();
        stateSubsystems.add(ledsSystem);
        stateSubsystems.add(elevatorSystem);
        stateSubsystems.add(armSystem);
        stateSubsystems.add(driveController);
        //stateSubsystems.add(climberSystem);
        subsystems = new ArrayList<>();
        subsystems.add(visionSystem);
        subsystems.add(autoSystem);
        createActions();
    }

    public void init(EnumCollection.OperatingMode mode) {
        currentOperatingMode = mode;
        interruptAction();

        switch (mode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                break;
            case TELEOP:
                driveController.setState(DriveController.DriverStates.DRIVER);
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
                break;
            case AUTONOMOUS:
                if (currentAction != null) {
                    if (isSequentialAction(currentAction)) {
                        if (currentSequentialAction != currentAction) {
                            currentSequentialAction = currentAction;
                            sequentialActionStatus = new ArrayList<>();
                            SubsystemSetting[] subsystemSettings = getSequentialActionSubsystemSettings(currentSequentialAction);
                            for (SubsystemSetting subsystemSetting : subsystemSettings) {
                                sequentialActionStatus.add(new SequentialActionStatus(false, subsystemSetting));
                            }
                        }
                        SubsystemSetting[] subsystemSettings = getSequentialActionSubsystemSettings(currentSequentialAction);

                        for (int i = 0; i < subsystemSettings.length; i++) {
                            SubsystemSetting setting = subsystemSettings[i];
                            if ((i == 0 || sequentialActionStatus.get(i - 1).stateHasBeenMet)) {
                                setting.subsystem.setState(setting.state);
                            }

                            if (!sequentialActionStatus.get(i).stateHasBeenMet && setting.subsystem.getCurrentState() == setting.state) {
                                sequentialActionStatus.get(i).stateHasBeenMet = setting.subsystem.matchesState();
                            }
                        }
                        break;
                    } else if (currentSequentialAction != null) {
                        currentSequentialAction = null;
                        sequentialActionStatus = new ArrayList<>();
                    }


                    SubsystemSetting[] subsystemSettings = getSubsystemSettingsFromAction(currentAction);
                    for (SubsystemSetting subsystemSetting : subsystemSettings) {
                        if (subsystemSetting.subsystem.getCurrentState() == subsystemSetting.state) continue;

                        subsystemSetting.subsystem.setState(subsystemSetting.state);
                    }
                }
                break;
            case TELEOP:
                processDriverInputs();
                Logger.recordOutput("RobotController/hasActionFinished", hasActionFinished());
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
            Logger.recordOutput(subsystem.name + "/MatchesState", subsystem.matchesState());
            Logger.recordOutput(subsystem.name + "/currentState", subsystem.getCurrentState().toString());
        }
    }

    private SubsystemSetting[] getSequentialActionSubsystemSettings(Action action) {
        SubsystemSetting[] temp = getSubsystemSettingsFromAction(action);
        SubsystemSetting[] subsystemSettings = new SubsystemSetting[temp.length - 1];
        System.arraycopy(temp, 1, subsystemSettings, 0, subsystemSettings.length);
        return subsystemSettings;
    }

    private void processDriverInputs() {
        DriverInput.InputHolder inputHolder = driverInput.getDriverInputs();
        mostRecentInputHolder = inputHolder;
        driverDriving(inputHolder.driverLeftJoystickPosition, inputHolder.driverRightJoystickPosition);
        List<StateSubsystem> setSubsystems = new ArrayList<>();

        if (inputHolder.resetNavX) {
            driveController.resetNavX();
        }

        if (inputHolder.sequentialAction != null) {
            if (currentSequentialAction != inputHolder.sequentialAction) {
                currentSequentialAction = inputHolder.sequentialAction;
                sequentialActionStatus = new ArrayList<>();
                SubsystemSetting[] subsystemSettings = getSequentialActionSubsystemSettings(currentSequentialAction);
                for (SubsystemSetting subsystemSetting : subsystemSettings) {
                    sequentialActionStatus.add(new SequentialActionStatus(false, subsystemSetting));
                }
            }

            SubsystemSetting[] subsystemSettings = getSequentialActionSubsystemSettings(inputHolder.sequentialAction);

            for (int i = 0; i < subsystemSettings.length; i++) {
                SubsystemSetting setting = subsystemSettings[i];
                if ((i == 0 || sequentialActionStatus.get(i - 1).stateHasBeenMet)) {
                    if (sequentialActionStatus.size() == subsystemSettings.length && sequentialActionStatus.get(i).stateHasBeenMet) {
                        setSubsystems.add(setting.subsystem);
                    } else {
                        setting.subsystem.setState(setting.state);
                        setSubsystems.add(setting.subsystem);
                    }
                }

                if (!sequentialActionStatus.get(i).stateHasBeenMet && setting.subsystem.getCurrentState() == setting.state) {
                    sequentialActionStatus.get(i).stateHasBeenMet = setting.subsystem.matchesState();
                }
            }
        } else if (currentSequentialAction != null) {
            currentSequentialAction = null;
            sequentialActionStatus = new ArrayList<>();
        }

        for (int i = 0; i < inputHolder.requestedSubsystemSettings.size(); i++) {
            SubsystemSetting setting = inputHolder.requestedSubsystemSettings.get(i);

            setting.subsystem.setState(setting.state);
            setSubsystems.add(setting.subsystem);
        }

        for (StateSubsystem subsystem : stateSubsystems) {
            if (subsystem.getCurrentState() != subsystem.defaultState && !setSubsystems.contains(subsystem)) {
                if (subsystem.name == "DriveController" && subsystem.getCurrentState() != DriveController.DriverStates.DRIVER) {
                    subsystem.setState(DriveController.DriverStates.DRIVER);
                } else if (subsystem.name == "DriveController" && subsystem.getCurrentState() == DriveController.DriverStates.DRIVER) {
                    return;
                }
                subsystem.restoreToDefaultState();
            }
        }
    }

    private void driverDriving(Translation2d translation, Translation2d rotation) {
        int flipForRed = isRed() ? -1 : 1;

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

    public void updateLocalization(EstimatedRobotPose visionEstimate, PhotonPipelineResult pipelineResult) {
        Pose2d estimatedPose = visionEstimate.estimatedPose.toPose2d();
        driveController.updatePoseUsingVisionEstimate(
                estimatedPose,
                visionEstimate.timestampSeconds,
                visionSystem.getEstimationStdDevs(
                        estimatedPose,
                        pipelineResult
                )
        );
    }


    public DriverInput.ReefCenteringSide getCenteringSide() {
        if (currentOperatingMode == EnumCollection.OperatingMode.AUTONOMOUS) {
            if (currentAction == null) {
                return null;
            }
            return switch (currentAction) {
                case CENTER_REEF_LEFT -> DriverInput.ReefCenteringSide.LEFT;
                case CENTER_REEF_RIGHT -> DriverInput.ReefCenteringSide.RIGHT;
                default -> null;
            };
        }

        return mostRecentInputHolder.centeringSide;
    }

    public boolean shouldBeCentering() {
        return getCenteringSide() != null && getCenteringDistance() < 1.5;
    }

    // Auto
    public void setAutoStep(AutoStep autoStep) {
        if (autoStep == null) return;

        switch (autoStep.getStepType()) {
            case ACTION:
                setAction(autoStep.getAction());
                break;
            case PATH:
                driveController.setAutoPath(autoStep.getPath());
                driveController.setState(DriveController.DriverStates.PATH);
                break;
            case PATH_AND_ACTION:
                setAction(autoStep.getAction());
                driveController.setAutoPath(autoStep.getPath());
                break;
        }
    }

    public boolean isSequentialAction(Action action) {
        if (action == null) return false;
        return getSubsystemSettingsFromAction(action)[0].reliesOnOthers;
    }

    public boolean hasFinishedAutoStep() {
        AutoStep currentAutoStep = autoSystem.getCurrentStep();
        if (currentAutoStep == null) return true;

        switch (currentAutoStep.getStepType()) {
            case ACTION:
                return hasActionFinished();
            case PATH:
                return driveController.matchesState();
            case PATH_AND_ACTION:
                return driveController.matchesState() && hasActionFinished();
        }

        return true;
    }

    private boolean hasActionFinished() {
        SubsystemSetting[] subsystemSettings = getSubsystemSettingsFromAction(currentAction);
        if (subsystemSettings == null) return false;
        if (isSequentialAction(currentAction)) {
            if (sequentialActionStatus == null) return false;
            for (SequentialActionStatus sequentialActionStatus : sequentialActionStatus) {
                if (!sequentialActionStatus.stateHasBeenMet) {
                    return false;
                }
            }
        } else {
            if (currentAction == Action.CENTER_REEF_LEFT || currentAction == Action.CENTER_REEF_RIGHT) {
                return driveController.hasFinishedCentering();
            }

            for (SubsystemSetting subsystemSetting : subsystemSettings) {
                if (!subsystemSetting.subsystem.matchesState()) {
                    return false;
                }
            }
        }
        return true;
    }

    public Action getCurrentAction() {
        return currentAction;
    }

    public void setAction(Action action) {
        currentAction = action;
    }

    public void interruptAction() {
        driveController.setAutoPath(null);
        currentAction = null;

        for (StateSubsystem stateSubsystem : stateSubsystems) {
            stateSubsystem.restoreToDefaultState();
        }
    }

    public enum Action {
        WAIT,
        LEDs_RED,
        LEDs_BLUE,
        EXPECTED_LED_FAIL,
        LEDs_GREEN,
        CENTER_REEF_LEFT,
        CENTER_REEF_RIGHT,
        L1,
        L2,
        L3,
        L4,
        COLLECT,
        MANUAL_ELEVATOR_UP,
        MANUAL_ELEVATOR_DOWN,
        CLIMBER_DOWN,
        CLIMBER_UP,
    }

    public void createActions() {
        defineAction(Action.LEDs_BLUE,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.BLUE, 3));
        defineAction(Action.LEDs_RED,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.RED, 3));
        defineAction(Action.LEDs_GREEN,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.GREEN, 4));
        defineAction(Action.EXPECTED_LED_FAIL,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.EXPECTED_FAIL, 0));
        defineAction(Action.L4,
                new SubsystemSetting(true),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L4, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.WAIT, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.L4, 5)
                );
        defineAction(Action.COLLECT,
                new SubsystemSetting(true),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.COLLECT_HIGH, 2),
                new SubsystemSetting(armSystem, Arm.ArmState.COLLECT, 2),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.COLLECT_LOW, 2)
                );
        defineAction(Action.MANUAL_ELEVATOR_DOWN,
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.MANUAL_DOWN, 2));
        defineAction(Action.MANUAL_ELEVATOR_UP,
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.MANUAL_UP, 3));
        defineAction(Action.CENTER_REEF_LEFT,
                new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING, 5));
        defineAction(Action.CENTER_REEF_RIGHT,
                new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING, 5));
/*        defineAction(Action.CLIMBER_DOWN,
                new SubsystemSetting(climberSystem, BaseClimber.ClimberState.DOWN, 3));
        defineAction(Action.CLIMBER_UP,
                new SubsystemSetting(climberSystem, BaseClimber.ClimberState.UP, 4));*/
    }

    public boolean isSafeForArmToMove() {
        if (elevatorSystem.getCurrentState() == elevatorSystem.defaultState) {
            return elevatorSystem.matchesState();
        }

        return false;
    }

    public double getCenteringDistance() {
        int apriltagId;
        double cameraDistance;

        if (getCenteringSide() == DriverInput.ReefCenteringSide.LEFT) {
            apriltagId = visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_LEFT);
            if (apriltagId != -1) {
                cameraDistance = visionSystem.getCameraDistanceX(Vision.Camera.FRONT_LEFT, apriltagId);
                return cameraDistance;
            }
        }

        return 1781;
    }


    public ArrayList<StateSubsystem> getFailedSubsystems() {
        ArrayList<StateSubsystem> failedSubsystems = new ArrayList<>();
        for (StateSubsystem stateSubsystem : stateSubsystems) {
            if (!stateSubsystem.matchesState()) {
                failedSubsystems.add(stateSubsystem);
            }
        }

        return failedSubsystems;
    }

    public SubsystemSetting[] getSubsystemSettingsFromAction(Action action) {
        if (getCenteringSide() != null
                && currentOperatingMode == EnumCollection.OperatingMode.AUTONOMOUS
                && autoSystem.getCurrentAutoStep().getPath() != null) {
            return EEUtil.insertElementAtIndex(actionMap.get(action), new SubsystemSetting(driveController, DriveController.DriverStates.PATH, 5), 1);
        }

        return actionMap.get(action);
    }

    public void defineAction(Action action, SubsystemSetting... settings) {
        actionMap.put(action, settings);
    }

    public static boolean isRed() {
        try {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }
        catch (NoSuchElementException e) {
            return false;
        }

    }

    private static class SequentialActionStatus {
        public boolean stateHasBeenMet = false;
        private final SubsystemSetting subsystemSetting;

        public SequentialActionStatus(boolean stateHasBeenMet, SubsystemSetting subsystemSetting) {
            this.stateHasBeenMet = stateHasBeenMet;
            this.subsystemSetting = subsystemSetting;
        }

        public SubsystemSetting getSubsystemSetting() {
            return subsystemSetting;
        }
    }

    public static class SubsystemSetting {
        public StateSubsystem subsystem;
        public StateSubsystem.SubsystemState state;
        public int weight;
        public boolean reliesOnOthers = false;

        public SubsystemSetting(boolean reliesOnOthers) {
            this.reliesOnOthers = reliesOnOthers;
        }

        public SubsystemSetting(StateSubsystem subsystem, StateSubsystem.SubsystemState state, int weight) {
            this.subsystem = subsystem;
            this.state = state;
            this.weight = weight;
        }

        @Override
        public String toString() {
            return new StringBuilder().append("Relies On Other: ").append(reliesOnOthers).append(" State: ").append(state).append(" Subsytems: ").append(subsystem).toString();
        }
    }
}
