package tech.lindblom.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import tech.lindblom.utils.Constants;

import java.util.ArrayList;

public class DriverInput {
    private RobotController robotController;
    private static int lastButton = -1;
    private static XboxController xBoxController = new XboxController(0);
    private static GenericHID buttonBoard = new GenericHID(1);

    private Control[] controlList;

    DriverInput(RobotController robotController) {
        this.robotController = robotController;
        this.controlList = new Control[] {
                new Control(0, "X", RobotController.Action.L4),
                new Control(0, "Y", RobotController.Action.COLLECT),
                new Control(0, "B", RobotController.Action.MANUAL_ELEVATOR_DOWN),
                new Control(0, "A", RobotController.Action.MANUAL_ELEVATOR_UP),
                new Control(0, "LB", RobotController.Action.CENTER_REEF_LEFT),
                new Control(0, "RB", RobotController.Action.CENTER_REEF_RIGHT),
                new Control(1,"button1", RobotController.Action.CENTER_REEF_LEFT_L4),
                new Control(1,"button2", RobotController.Action.CENTER_REEF_RIGHT_L4),
                //new Control(1,"button3", RobotController.Action.CENTER_REEF_LEFT_L3),
                //new Control(1,"button4", RobotController.Action.CENTER_REEF_RIGHT_L3),
                //new Control(1,"button5", RobotController.Action.CENTER_REEF_LEFT_L2),
                //new Control(1,"button6", RobotController.Action.CENTER_REEF_RIGHT_L2),
                //new Control(1,"button7", RobotController.Action.CENTER_REEF_LEFT_L1),
                //new Control(1,"button8", RobotController.Action.CENTER_REEF_RIGHT_L1),
                new Control(0, "DPAD_UP", RobotController.Action.CLIMBER_LATCH_RELEASE),
        };
    }

    public InputHolder getDriverInputs() {
        InputHolder driverInputHolder = new InputHolder();

        driverInputHolder.driverLeftJoystickPosition = getControllerJoyAxis(ControllerSide.LEFT, 0);
        driverInputHolder.driverRightJoystickPosition = getControllerJoyAxis(ControllerSide.RIGHT, 0);

        driverInputHolder.resetNavX = getButton("START", 0);

        ArrayList<RobotController.SubsystemSetting> subsystemSettings = new ArrayList<>();

        for (int i = 0; i < controlList.length; i++) {
            Control control = controlList[i];

            if (control.getButtonValue()) {
                if (control.requestedAction == RobotController.Action.CENTER_REEF_LEFT) {
                    driverInputHolder.centeringSide = ReefCenteringSide.LEFT;
                } else if (control.requestedAction == RobotController.Action.CENTER_REEF_RIGHT) {
                    driverInputHolder.centeringSide = ReefCenteringSide.RIGHT;
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
        var selectedController = (XboxController) xBoxController;
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
        getButton();
        switch (buttonName) {
            case "START":
                return xBoxController.getStartButton();
            case "A":
                return xBoxController.getAButton();
            case "B":
                return xBoxController.getBButton();
            case "X":
                return xBoxController.getXButton();
            case "Y":
                return xBoxController.getYButton();
            case "LB":
                return xBoxController.getLeftBumperButton();
            case "RB":
                return xBoxController.getRightBumperButton();
            case "button1":
                return (lastButton == 1 || buttonBoard.getRawButtonPressed(1)) && getButton("X", 0);
            case "button2":
                return (lastButton == 2 || buttonBoard.getRawButtonPressed(2)) && getButton("X", 0);
            case "button3":
                return (lastButton == 3 || buttonBoard.getRawButtonPressed(3)) && getButton("X", 0);
            case "button4":
                return (lastButton == 4 || buttonBoard.getRawButtonPressed(4)) && getButton("X", 0);
            case "button5":
                return (lastButton == 5 || buttonBoard.getRawButtonPressed(5)) && getButton("X", 0);
            case "button6":
                return (lastButton == 6 || buttonBoard.getRawButtonPressed(6)) && getButton("X", 0);
            case "button7":
                return (lastButton == 7 || buttonBoard.getRawButtonPressed(7)) && getButton("X", 0);
            case "button8":
                return (lastButton == 8 || buttonBoard.getRawButtonPressed(8)) && getButton("X", 0);
            case "DPAD_UP":
                return xBoxController.getPOV() == 0;
        }

        return false;
    }

    public static int getButton() {
        int button = -1;
        for (int i = 1;  i <= 8; i++) {
          if (buttonBoard.getRawButtonPressed(i)) {
            button = i;
          }
        }

        lastButton = button;
        return button;
      }

    class InputHolder {
        public ArrayList<RobotController.SubsystemSetting> requestedSubsystemSettings;
        public RobotController.Action sequentialAction;

        Translation2d driverLeftJoystickPosition;
        Translation2d driverRightJoystickPosition;

        public ReefCenteringSide centeringSide = null;

        boolean resetNavX;
    }

    public enum ReefCenteringSide {
        LEFT, RIGHT, CENTER
    }

    public enum ControllerSide {
        LEFT, RIGHT
    }
}
