package tech.lindblom.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import tech.lindblom.control.RobotController.Action;
import tech.lindblom.utils.Constants;

import java.util.ArrayList;

public class DriverInput {
    private RobotController robotController;
    private static int lastButton = -1;
    private static XboxController[] controllers = new XboxController[] {
            new XboxController(0),
            new XboxController(1)
    };

    private Control[] controlList;

    DriverInput(RobotController robotController) {
        this.robotController = robotController;
        this.controlList = new Control[] {
                new Control(0, "RB", Action.CRADLE_COLLECT),
                new Control(0, "DPAD_RIGHT", Action.MANUAL_ARM_UP),
                new Control(0, "DPAD_LEFT", Action.MANUAL_ARM_DOWN),
                new Control(0, "DPAD_UP", Action.MANUAL_ELEVATOR_UP),
                new Control(0, "DPAD_DOWN", Action.MANUAL_ELEVATOR_DOWN),
                new Control(0, "LB", Action.THUMB_SPIN_IN),
                new Control(0, "X", Action.THUMB_SPIN_OUT),
                new Control(0, "RIGHT_TRIGGER", Action.INHIBIT_DRIVE),
                new Control(1, "DPAD_LEFT", Action.GROUND_COLLECT_ALGAE),
                new Control(1, "DPAD_UP", Action.CLIMBER_UP),
                new Control(1, "DPAD_DOWN", Action.CLIMBER_DOWN),
                new Control(1, "RB", Action.CENTER_REEF_RIGHT),  //Ignore if pole already found and still holding down button
                new Control(1, "LB", Action.CENTER_REEF_LEFT), //Ignore if pole already found and still holding down button
                new Control(1, "BACK", Action.CLIMBER_LATCH_RELEASE),
                new Control(1, "A", Action.L4),
                new Control(1, "B", Action.L3),
                new Control(1, "X", Action.L2),
                new Control(1, "Y", Action.REMOVE_ALGAE),
                new Control(1, "LEFT_JOYSTICK_BUTTON", Action.HIGH_HOLD_ALGAE),
                new Control(1, "RIGHT_JOYSTICK_BUTTON", Action.REEF_COLLECT_ALGAE)
        };
    }

    public InputHolder getDriverInputs() {
        InputHolder driverInputHolder = new InputHolder();

        driverInputHolder.driverLeftJoystickPosition = getControllerJoyAxis(ControllerSide.LEFT, 0);
        driverInputHolder.driverRightJoystickPosition = getControllerJoyAxis(ControllerSide.RIGHT, 0);

        driverInputHolder.orientFieldToRobot = getButton("START", 0);
        driverInputHolder.toggleManualControl = getButton("B", 0);
        driverInputHolder.armConfirm = getButton("BACK", 0);

        ArrayList<RobotController.SubsystemSetting> subsystemSettings = new ArrayList<>();

        for (int i = 0; i < controlList.length; i++) {
            Control control = controlList[i];

            if (control.getButtonValue()) {
                if (control.requestedAction == RobotController.Action.CENTER_REEF_LEFT) {
                    driverInputHolder.centeringSide = ReefCenteringSide.LEFT;
                } else if (control.requestedAction == RobotController.Action.CENTER_REEF_RIGHT) {
                    driverInputHolder.centeringSide = ReefCenteringSide.RIGHT;
                } else if (control.requestedAction == Action.REEF_COLLECT_ALGAE) {
                    driverInputHolder.centeringSide = ReefCenteringSide.CENTER;
                }

                RobotController.SubsystemSetting[] subsystemSettingsFromAction = robotController.getSubsystemSettingsFromAction(control.requestedAction);
                if (subsystemSettingsFromAction == null) break;

                if (subsystemSettingsFromAction[0].reliesOnOthers) {
                    driverInputHolder.sequentialAction = control.requestedAction;
                    break;
                }

                for (RobotController.SubsystemSetting subsystemSettingFromAction : subsystemSettingsFromAction) {
                    for (RobotController.SubsystemSetting subsystemSetting : subsystemSettings) {
                        if (driverInputHolder.sequentialAction != null) {
                            for (RobotController.SubsystemSetting sequentialActionSetting : robotController.getSubsystemSettingsFromAction(driverInputHolder.sequentialAction)) {
                                if (sequentialActionSetting.subsystem == subsystemSetting.subsystem) break;
                            }
                        }

                        if (subsystemSetting.subsystem == subsystemSettingFromAction.subsystem && subsystemSetting.weight < subsystemSettingFromAction.weight) {
                            subsystemSettings.add(subsystemSettingFromAction);
                            subsystemSettings.remove(subsystemSetting);
                            break;
                        }
                    }

                    subsystemSettings.add(subsystemSettingFromAction);
                }
            }
        }

        driverInputHolder.requestedSubsystemSettings = subsystemSettings;

        return driverInputHolder;
    }

    public Translation2d getControllerJoyAxis(ControllerSide side, int controllerIndex) {
        var selectedController = (XboxController) controllers[controllerIndex];
        double x;
        double y;

        if (!selectedController.isConnected()) {
            return new Translation2d();
        }

        if (side == ControllerSide.LEFT) {
            x = selectedController.getLeftX();
            y = selectedController.getLeftY();
        } else {
            x = selectedController.getRightX();
            y = selectedController.getRightY();
        }

        if(Math.abs(x) <= Constants.Controls.DEADZONE) {
            x = 0;
        }
        else {
            if (x < 0) {
                x = Math.abs(x);
                x -= Constants.Controls.DEADZONE;
                x *= 1/(1 - Constants.Controls.DEADZONE);
                x *= -1;
            }
            else {
                x -= Constants.Controls.DEADZONE;
                x *= 1/(1 - Constants.Controls.DEADZONE);
            }
        }

        if(Math.abs(y) <= Constants.Controls.DEADZONE) {
            y = 0;
        }
        else {
            if (y < 0) {
                y = Math.abs(y);
                y -= Constants.Controls.DEADZONE;
                y *= 1/(1 - Constants.Controls.DEADZONE);
                y *= -1;
            }
            else {
                y -= Constants.Controls.DEADZONE;
                y *= 1/(1 - Constants.Controls.DEADZONE);
            }
        }

        return new Translation2d(x, y);
    }

    private class Control {
        private final int controllerIndex;
        private final String inputName;
        private final RobotController.Action requestedAction;

        Control(int controllerIndex, String input, RobotController.Action requestedAction) {
            this.controllerIndex = controllerIndex;
            this.inputName = input;

            this.requestedAction = requestedAction;
        }

        boolean getButtonValue() {
            return DriverInput.getButton(inputName, controllerIndex);
        }


        enum InputType {
            BUTTON
        }
    }

    public static boolean getButton(String buttonName, int controllerIndex) {
        //getButton();
        switch (buttonName) {
            case "START":
                return controllers[controllerIndex].getStartButton();
            case "A":
                return controllers[controllerIndex].getAButton();
            case "B":
                return controllers[controllerIndex].getBButton();
            case "X":
                return controllers[controllerIndex].getXButton();
            case "Y":
                return controllers[controllerIndex].getYButton();
            case "LB":
                return controllers[controllerIndex].getLeftBumperButton();
            case "RB":
                return controllers[controllerIndex].getRightBumperButton();
            case "DPAD_UP":
                return controllers[controllerIndex].getPOV() == 0;
            case "DPAD_RIGHT":
                return controllers[controllerIndex].getPOV() == 90;
            case "DPAD_DOWN":
                return controllers[controllerIndex].getPOV() == 180;
            case "DPAD_LEFT":
                return controllers[controllerIndex].getPOV() == 270;
            case "BACK":
                    return controllers[controllerIndex].getBackButton();
            case "LEFT_JOYSTICK_BUTTON":
                return controllers[controllerIndex].getLeftStickButton();
            case "RIGHT_JOYSTICK_BUTTON":
                return controllers[controllerIndex].getRightStickButton();
            case "RIGHT_TRIGGER":
                return controllers[controllerIndex].getRightTriggerAxis() > 0.9;
        }
        return false;
    }

    class InputHolder {
        public ArrayList<RobotController.SubsystemSetting> requestedSubsystemSettings;
        public RobotController.Action sequentialAction;

        Translation2d driverLeftJoystickPosition;
        Translation2d driverRightJoystickPosition;

        public ReefCenteringSide centeringSide = null;
        public boolean toggleManualControl = false;
        public boolean armConfirm = false;

        boolean orientFieldToRobot;
    }

    public enum ReefCenteringSide {
        LEFT, RIGHT, CENTER
    }

    public enum ControllerSide {
        LEFT, RIGHT
    }
}
