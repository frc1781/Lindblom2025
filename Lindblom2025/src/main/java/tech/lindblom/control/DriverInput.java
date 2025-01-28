package tech.lindblom.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.utils.Constants;

import java.util.ArrayList;

public class DriverInput {
    private RobotController robotController;
    private static XboxController[] controllers = new XboxController[] {
            new XboxController(0)
    };

    private Control[] controlList;

    DriverInput(RobotController robotController) {
        this.robotController = robotController;
        this.controlList = new Control[] {
                new Control(0, "A", RobotController.Action.LEDs_RED),
                new Control(0, "B", RobotController.Action.LEDs_GREEN)
        };
    }

    public InputHolder getDriverInputs() {
        InputHolder driverInputHolder = new InputHolder();

        driverInputHolder.driverLeftJoystickPosition = getControllerJoyAxis(ControllerSide.LEFT, 0);
        driverInputHolder.driverRightJoystickPosition = getControllerJoyAxis(ControllerSide.RIGHT, 0);

        driverInputHolder.resetNavX = getButton("START", 0);

        ArrayList<RobotController.SubsystemSetting> subsystemSettings = new ArrayList<>();

        if (getButton("LB", 0)) {
            driverInputHolder.centeringSide = ReefCenteringSide.LEFT;
        } else if (getButton("RB", 0)) {
            driverInputHolder.centeringSide = ReefCenteringSide.RIGHT;
        }

        for (int i = 0; i < controlList.length; i++) {
            Control control = controlList[i];

            if (control.getButtonValue()) {
                RobotController.SubsystemSetting[] subsystemSettingsFromAction = robotController.getSubsystemSettingsFromAction(control.requestedAction);
                if (subsystemSettingsFromAction == null) break;

                for (RobotController.SubsystemSetting subsystemSettingFromAction : subsystemSettingsFromAction) {
                    for (RobotController.SubsystemSetting subsystemSetting : subsystemSettings) {
                        if (subsystemSetting.subsystem == subsystemSettingFromAction.subsystem && subsystemSetting.weight < subsystemSettingFromAction.weight) {
                            subsystemSettings.add(subsystemSetting);
                            subsystemSettings.remove(subsystemSettingFromAction);
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
        switch (buttonName) {
            case "START":
                return controllers[controllerIndex].getStartButton();
            case "A":
                return controllers[controllerIndex].getAButton();
            case "B":
                return controllers[controllerIndex].getBButton();
            case "LB":
                return controllers[controllerIndex].getLeftBumperButton();
            case "RB":
                return controllers[controllerIndex].getRightBumperButton();
        }

        return false;
    }

    class InputHolder {
        public ArrayList<RobotController.SubsystemSetting> requestedSubsystemSettings;
        public ReefCenteringSide centeringSide = null;

        Translation2d driverLeftJoystickPosition;
        Translation2d driverRightJoystickPosition;

        boolean resetNavX;
    }

    public enum ReefCenteringSide {
        LEFT, RIGHT, CENTER
    }

    public enum ControllerSide {
        LEFT, RIGHT
    }
}
