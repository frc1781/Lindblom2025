package tech.lindblom.control;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.subsystems.arm.Arm;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.routines.CenterOneCoral;
import tech.lindblom.subsystems.auto.routines.Collect;
import tech.lindblom.subsystems.auto.routines.LeftFourCoral;
import tech.lindblom.subsystems.auto.routines.LeftOneCoral;
import tech.lindblom.subsystems.auto.routines.LeftThreeCoral;
import tech.lindblom.subsystems.auto.routines.RightFourCoral;
import tech.lindblom.subsystems.auto.routines.RightOneCoral;
import tech.lindblom.subsystems.auto.routines.RightThreeCoral;
import tech.lindblom.subsystems.auto.routines.TestRoutine;
import tech.lindblom.subsystems.climber.BaseClimber;
import tech.lindblom.subsystems.climber.Climber;
import tech.lindblom.subsystems.climber.ClimberSim;
import tech.lindblom.subsystems.conveyor.Conveyor;
import tech.lindblom.subsystems.thumb.Thumb;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.elevator.Elevator;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.subsystems.led.LEDs.LEDState;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection;

// The robot controller, controls robot.
public class RobotController {
    public DriveController driveController;
    public Vision visionSystem;
    public Auto autoSystem;
    public LEDs ledsSystem;
    public Elevator elevatorSystem;
    public Arm armSystem;
    public BaseClimber climberSystem;
    public Conveyor conveyorSystem;
    public Thumb thumbSystem;

    DriverInput driverInput;

    public Timer autoTimer = new Timer();

    private Action currentAction;
    private final ArrayList<StateSubsystem> stateSubsystems;
    private final ArrayList<Subsystem> subsystems;

    private final SlewRateLimiter xControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter yControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter rotControllerLimiter = new SlewRateLimiter(Constants.Drive.DRIVER_ROTATION_RATE_LIMIT);

    private final HashMap<Action, SubsystemSetting[]> actionMap = new HashMap<>();

    private DriverInput.InputHolder mostRecentInputHolder;
    private ArrayList<SequentialActionStatus> sequentialActionStatus = new ArrayList<>();
    private Action currentSequentialAction = null;
    private EnumCollection.OperatingMode currentOperatingMode = null;

    private boolean manualControlMode = false;
    private boolean toggleSwitch = false;

    public RobotController() {
        driveController = new DriveController(this);
        autoSystem = new Auto(this,
                new TestRoutine(),
                new Collect(),
                new LeftThreeCoral(),
                new RightThreeCoral(),
                new RightFourCoral(),
                new LeftFourCoral(),
                new LeftOneCoral(),
                new RightOneCoral(),
                new CenterOneCoral()
        );
        visionSystem = new Vision(this);
        ledsSystem = new LEDs(this);
        thumbSystem = new Thumb();
        elevatorSystem = new Elevator(this);
        armSystem = new Arm(this);
        conveyorSystem = new Conveyor(this);
        driverInput = new DriverInput(this);
        if (RobotBase.isReal()) {
            climberSystem = new Climber();
        } else {
            climberSystem = new ClimberSim();
        }
        stateSubsystems = new ArrayList<>();
        stateSubsystems.add(thumbSystem);
        stateSubsystems.add(elevatorSystem);
        stateSubsystems.add(armSystem);
        stateSubsystems.add(climberSystem);
        stateSubsystems.add(driveController);
        stateSubsystems.add(conveyorSystem);
        subsystems = new ArrayList<>();
        subsystems.add(visionSystem);
        subsystems.add(autoSystem);
        subsystems.add(thumbSystem);
        subsystems.add(ledsSystem);
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
                ledsSystem.setState(LEDState.SYNC);
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
        Logger.recordOutput("RobotController/isSafeForArmToLeaveIdle", isSafeForArmToLeaveIdle());
        Logger.recordOutput("RobotController/isSafeForElevatorStage2toMove", isSafeForElevatorStage2toMove());
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
                                if (sequentialActionStatus.size() != subsystemSettings.length || !sequentialActionStatus.get(i).stateHasBeenMet) {
                                    setting.subsystem.setState(setting.state);
                                }
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
                if (driveController.getCurrentState() == DriveController.DriverStates.IDLE) {
                    driveController.setState(DriveController.DriverStates.DRIVER);
                }

                processDriverInputs();
                if (driveController.reefPoleDetected()) {
                    ledsSystem.setState(LEDState.GREEN);
                }
                else if (driveController.readyForCentering()) {
                    ledsSystem.setState(LEDState.RED);
                }
                else if (conveyorSystem.cradleHasCoral()) {
                    ledsSystem.setState(LEDState.YELLOW);
                }
                else if (armSystem.hasCoral()) {
                    ledsSystem.setState(LEDState.BLUE);   
                }
                else {
                    ledsSystem.setState(LEDState.OPERATING_COLOR);
                }
                
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
        Logger.recordOutput("RobotController/DriverCenteringSide", mostRecentInputHolder.centeringSide);
        driverDriving(inputHolder.driverLeftJoystickPosition, inputHolder.driverRightJoystickPosition);
        List<StateSubsystem> setSubsystems = new ArrayList<>();

        if (inputHolder.toggleManualControl && !toggleSwitch) {
            manualControlMode = !manualControlMode;
        }

        Logger.recordOutput("RobotController/manualControlMode", manualControlMode);

        toggleSwitch = inputHolder.toggleManualControl;

        if (inputHolder.orientFieldToRobot) {
            driveController.orientFieldToRobot();
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
            if (subsystem.getCurrentState() != subsystem.getDefaultState() && !setSubsystems.contains(subsystem)) {
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
        Logger.recordOutput("Driver/isRed", isRed());

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
                case CENTER_REEF_LEFT, CENTER_REEF_LEFT_L4:
                    yield DriverInput.ReefCenteringSide.LEFT;
                case CENTER_REEF_RIGHT, CENTER_REEF_RIGHT_L4:
                    yield DriverInput.ReefCenteringSide.RIGHT;
                default:
                    yield null;
            };
        }

        return mostRecentInputHolder.centeringSide;
    }

    public boolean shouldBeCentering() {
        return getCenteringSide() != null && getCenteringDistance() < 1;
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
                if (!isSequentialAction(autoStep.getAction())) {
                    driveController.setState(DriveController.DriverStates.PATH);
                }
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
        if (currentAction == null) return false;
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
            if (getCenteringSide() != null) {
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

    public boolean elevatorInConveyPosition() {
        return (elevatorSystem.getCurrentState() == Elevator.ElevatorState.SAFE && elevatorSystem.matchesState()) || elevatorSystem.getFirstStagePosition() < 10;
    }

    public boolean isManualControlMode() {
        return manualControlMode;
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
        READY_FOP_POLE,
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
        CENTER_REEF_LEFT_L4,
        CENTER_REEF_RIGHT_L4,
        COLLECT,
        MANUAL_ELEVATOR_UP,
        MANUAL_ELEVATOR_DOWN,
        MANUAL_ARM_DOWN,
        MANUAL_ARM_UP,
        CLIMBER_DOWN,
        CLIMBER_UP,
        SPIN_IN,
        SPIN_OUT,
        FIND_POLE_LEFT,
        FIND_POLE_RIGHT,
        CLIMBER_LATCH_RELEASE,
        CONVEY_AND_COLLECT,
        READY_FOR_COLLECT,
        CENTER_REEF_CENTER
    }

    public void createActions(){
        defineAction(Action.READY_FOP_POLE,
                new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.POLE, 5));
        defineAction(Action.CENTER_REEF_CENTER,
                new SubsystemSetting(true),
                new SubsystemSetting(armSystem, Arm.ArmState.COLLECT, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.COLLECT_LOW, 5),
                new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING_CENTER, 5));
        defineAction(Action.LEDs_BLUE,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.BLUE, 3));
        defineAction(Action.LEDs_RED,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.RED, 3));
        defineAction(Action.LEDs_GREEN,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.GREEN, 4));
        defineAction(Action.EXPECTED_LED_FAIL,
                new SubsystemSetting(ledsSystem, LEDs.LEDState.EXPECTED_FAIL, 0));
                /* 
                 * 	Collect Action
                    1	Inhibit movement until any sensor detects a coral
                    1	Turn on conveyor if coral is detected by any beam break and not detected by the cradel front beam break
                    1	Move arm to collect if elevator and arm are in positions
                    2	move elevator to collect if cradel front beam break detects coral
                    3	move elevator to L4 if arm TOF detects coral
                    3	move arm to POLE if arm TOF detects coral and safe to do so
                    4   Leave in those states unless no longer detecting coral in Arm TOF
	                ANAYA: Check that using the correct beam break for turning off conveyor should be the front conveyor 
                    right I mean in theory you should be able to walk the coral through this hole thing right beam 
                    break happens here it lands in the trough or the conveyor has two different ones assuming this 
                    is up and a non blocking position then it should convey until neither of those is seen and 
                    the front one does see I wish at that point the hand should be able to grab the coral from 
                    this beam break and when it lifts the coral up we should see no Coral anywhere in the cradle 
                    and only see Coral in the time of flight and then after we place we should see no Coral in 
                    the time of flight that it then we should sense it all the way through the entire robot or 
                    we have that possibility I don't know how useful some of that is but well
                 */
        defineAction(Action.COLLECT,
                new SubsystemSetting(true),
                new SubsystemSetting(armSystem, Arm.ArmState.COLLECT, 2),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.COLLECT_LOW, 2)
                );

        defineAction(Action.MANUAL_ELEVATOR_DOWN,
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.MANUAL_DOWN, 2));

        defineAction(Action.MANUAL_ELEVATOR_UP,
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.MANUAL_UP, 3));

        defineAction(Action.MANUAL_ARM_DOWN,
                new SubsystemSetting(armSystem, Arm.ArmState.MANUAL_DOWN, 3));

        defineAction(Action.MANUAL_ARM_UP,
                new SubsystemSetting(armSystem, Arm.ArmState.MANUAL_UP, 3));

        defineAction(Action.CENTER_REEF_LEFT,
            new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING_LEFT, 6)
        );

        defineAction(Action.CENTER_REEF_RIGHT,
            new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING_RIGHT, 6)
        );

        defineAction(Action.L4,
            new SubsystemSetting(true),
            new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.POLE, 5),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L4, 5),
            new SubsystemSetting(armSystem, Arm.ArmState.L4, 5)
        );

        defineAction(Action.L3,
            new SubsystemSetting(true),
            new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.POLE, 5),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L3, 5),
            new SubsystemSetting(armSystem, Arm.ArmState.L3, 5)
        );

        defineAction(Action.L2,
            new SubsystemSetting(true),
                new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.POLE, 5),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L2, 5),
            new SubsystemSetting(armSystem, Arm.ArmState.L2, 5)
        );

        defineAction(Action.L1,
            new SubsystemSetting(true),
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L1, 5),
            new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
            new SubsystemSetting(armSystem, Arm.ArmState.L1, 5)
        );

        defineAction(Action.CENTER_REEF_LEFT_L4,
                new SubsystemSetting(true),
                new SubsystemSetting(driveController, DriveController.DriverStates.PATH, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.POLE, 5),
                new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING_LEFT, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L4, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.L4, 5)
                );
        defineAction(Action.CENTER_REEF_RIGHT_L4,
                new SubsystemSetting(true),
                new SubsystemSetting(driveController, DriveController.DriverStates.PATH, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.POLE, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.POLE, 5),
                new SubsystemSetting(driveController, DriveController.DriverStates.CENTERING_RIGHT, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.L4, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.L4, 5)
                );
        defineAction(Action.CONVEY_AND_COLLECT,
                new SubsystemSetting(true),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.SAFE, 5),
                new SubsystemSetting(armSystem, Arm.ArmState.COLLECT,5),
                new SubsystemSetting(conveyorSystem, Conveyor.ConveyorState.CONVEY, 5),
                new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.COLLECT_LOW, 5)
                );

        defineAction(Action.READY_FOR_COLLECT,
            new SubsystemSetting(elevatorSystem, Elevator.ElevatorState.SAFE, 0),
            new SubsystemSetting(armSystem, Arm.ArmState.COLLECT, 0));

        defineAction(Action.CLIMBER_LATCH_RELEASE,
                new SubsystemSetting(climberSystem, BaseClimber.ClimberState.RELEASE_LATCH, 5));

        defineAction(Action.CLIMBER_DOWN,
                  new SubsystemSetting(climberSystem, BaseClimber.ClimberState.DOWN, 3));

        defineAction(Action.CLIMBER_UP,
                new SubsystemSetting(climberSystem, BaseClimber.ClimberState.UP, 4));

         defineAction(Action.SPIN_IN,
                 new SubsystemSetting(thumbSystem, Thumb.ThumbState.SPIN_IN, 5));

         defineAction(Action.SPIN_OUT,
                 new SubsystemSetting(thumbSystem, Thumb.ThumbState.SPIN_OUT, 5));
    }

    public boolean isSafeForArmToLeaveIdle() {
        return (elevatorSystem.getSecondStagePosition() < 150);
    }

    public boolean isSafeForElevatorStage2toMove() {
        return armSystem.getPosition() > 40.0 && armSystem.getPosition() < 300;  //should never be this high except with gimble lock wrapping 
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
        } else {
            apriltagId = visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_RIGHT);
            if (apriltagId != -1) {
                cameraDistance = visionSystem.getCameraDistanceX(Vision.Camera.FRONT_RIGHT, apriltagId);
                return cameraDistance;
            }
        }

        return Constants.Vision.ERROR_CONSTANT;
    }

    public boolean isArmInPoleState() {
        return armSystem.getCurrentState() == Arm.ArmState.POLE && armSystem.matchesDesiredPosition();
    }

    public boolean isElevatorInPoleState() {
        return elevatorSystem.getCurrentState() == Elevator.ElevatorState.POLE && elevatorSystem.matchesPosition();
    }

    public Rotation2d getRobotHeading() {
        return driveController.getRobotHeading();
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
        if (currentOperatingMode == EnumCollection.OperatingMode.AUTONOMOUS
                && autoSystem.getCurrentStep().getPath() != null
                && actionMap.get(action)[0].reliesOnOthers
                ) {
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
