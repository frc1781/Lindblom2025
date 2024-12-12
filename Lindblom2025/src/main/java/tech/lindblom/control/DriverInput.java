package tech.lindblom.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import tech.lindblom.subsystems.arm.Arm;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.subsystems.scollector.Scollector;
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
                new Control(0, "A", new RobotController.SubsystemSetting[] {
                        new RobotController.SubsystemSetting(robotController.scollectorSystem, Scollector.ScollectorState.COLLECT, 2),
                        new RobotController.SubsystemSetting(robotController.armSystem, Arm.ArmState.COLLECT,2),
                }),
                new Control(0, "B", new RobotController.SubsystemSetting[]{
                        new RobotController.SubsystemSetting(robotController.ledsSystem, LEDs.LEDState.GREEN, 4)
                }),
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
                for (int t = 0; t < controlList[i].requestedSettings.length; t++) {
                    RobotController.SubsystemSetting controlSetting = controlList[i].requestedSettings[t];

                    for (int j = 0; j < subsystemSettings.size(); j++) {
                        RobotController.SubsystemSetting subsystemSetting = subsystemSettings.get(j);
                        if (subsystemSetting.subsystem == controlSetting.subsystem && subsystemSetting.weight < controlSetting.weight) {
                            subsystemSettings.add(controlSetting);
                            subsystemSettings.remove(subsystemSetting);
                            break;
                        }
                    }

                    subsystemSettings.add(controlSetting);
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
        private final RobotController.SubsystemSetting[] requestedSettings;

        Control(int controllerIndex, String input, RobotController.SubsystemSetting[] requestedSetting) {
            this.controllerIndex = controllerIndex;
            this.inputName = input;

            this.requestedSettings = requestedSetting;
        }

        boolean getButtonValue() {
            return DriverInput.getButton(inputName, controllerIndex);
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
        }

        return false;
    }

    class InputHolder {
        public ArrayList<RobotController.SubsystemSetting> requestedSubsystemSettings;

        Translation2d driverLeftJoystickPosition;
        Translation2d driverRightJoystickPosition;

        boolean resetNavX;
    }

    public enum ControllerSide {
        LEFT, RIGHT
    }
}
