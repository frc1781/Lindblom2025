package tech.lindblom.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import tech.lindblom.utils.Constants;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;

public class DriverInput {
    private GenericHID[] controllers = new GenericHID[] {
            new XboxController(0),
            new XboxController(1)
    };

    private Control[] controlList = new Control[] {
            new Control(this, 0, 0, DriverInput.class.getDeclaredMethod("getControllerJoyAxis", ControllerSide.class, int.class), Control.InputType.JOYSTICK, ControllerSide.LEFT),
            new Control(this, 0, 0, DriverInput.class.getDeclaredMethod("getControllerJoyAxis", ControllerSide.class, int.class), Control.InputType.JOYSTICK, ControllerSide.RIGHT)
    };


    DriverInput() throws NoSuchMethodException {

    }

    public Translation2d getDriverInputs() {
        for (int i = 0; i < controlList.length; i++) {
            Control control = controlList[i];
            return control.getJoystickPosition();
        }

        return new Translation2d(0, 0);
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
        private final DriverInput driverInput;
        private final int controllerIndex;
        private final int weight;
        private final Method inputFromController;
        private final InputType inputType;

        private final ControllerSide side;

        Control(DriverInput driverInput, int controllerIndex, int weight, Method input, InputType inputType) {
            this.driverInput = driverInput;
            this.controllerIndex = controllerIndex;
            this.weight = weight;
            this.inputFromController = input;
            this.inputType = inputType;

            this.side = null;
        }

        Control(DriverInput driverInput, int controllerIndex, int weight, Method input, InputType inputType, ControllerSide side) {
            this.driverInput = driverInput;
            this.controllerIndex = controllerIndex;
            this.weight = weight;
            this.inputFromController = input;
            this.inputType = inputType;

            this.side = side;
        }

        Translation2d getJoystickPosition() {
            if (inputType == InputType.JOYSTICK) {
                try {
                    Object joystick = inputFromController.invoke(driverInput, side, controllerIndex);
                    Translation2d joystickPosition = (Translation2d) joystick;
                    return joystickPosition;
                } catch (IllegalAccessException | InvocationTargetException e) {
                    e.printStackTrace();
                }
            }

            return new Translation2d();
        }

        boolean getButtonValue() {
            if (inputType == InputType.BUTTON) {
                try {
                    Object button = inputFromController.invoke(driverInput);
                    boolean buttonValue = (boolean) button;
                    return buttonValue;
                } catch (IllegalAccessException | InvocationTargetException e) {
                    e.printStackTrace();
                }
            }

            return false;
        }

        double getWeight() {
            return weight;
        }

        enum InputType {
            JOYSTICK, BUTTON, TRIGGER
        }
    }

    public enum ControllerSide {
        LEFT, RIGHT
    }
}
