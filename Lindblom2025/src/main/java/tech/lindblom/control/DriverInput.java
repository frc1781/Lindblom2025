package tech.lindblom.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;

public class DriverInput {
    private static XboxController[] controllers = new XboxController[] {
            new XboxController(0)
    };

    private Control[] controlList = new Control[] {

    };

    DriverInput() throws NoSuchMethodException {

    }

    public InputHolder getDriverInputs() {
        InputHolder driverInputHolder = new InputHolder();

        driverInputHolder.driverLeftJoystickPosition = getControllerJoyAxis(ControllerSide.LEFT, 0);
        driverInputHolder.driverRightJoystickPosition = getControllerJoyAxis(ControllerSide.RIGHT, 0);

        driverInputHolder.resetNavX = getButton("START", 0);

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
        private final DriverInput driverInput;
        private final int controllerIndex;
        private final int weight;
        private final String inputName;
        private final InputType inputType;
        private final StateSubsystem.SubsystemState requestedState;

        Control(DriverInput driverInput, int controllerIndex, int weight, String input, InputType inputType) {
            this.driverInput = driverInput;
            this.controllerIndex = controllerIndex;
            this.weight = weight;
            this.inputName = input;
            this.inputType = inputType;

            this.requestedState = null;
        }

        Control(DriverInput driverInput, int controllerIndex, int weight, String input, InputType inputType, StateSubsystem.SubsystemState requestedState) {
            this.driverInput = driverInput;
            this.controllerIndex = controllerIndex;
            this.weight = weight;
            this.inputName = input;
            this.inputType = inputType;

            this.requestedState = requestedState;
        }

        boolean getButtonValue() {
            if (inputType == InputType.BUTTON) {
                return DriverInput.getButton(inputName, controllerIndex);
            }

            return false;
        }

        double getWeight() {
            return weight;
        }

        enum InputType {
            BUTTON, TRIGGER
        }
    }

    public static boolean getButton(String buttonName, int controllerIndex) {
        switch (buttonName) {
            case "START":
                return controllers[controllerIndex].getStartButton();
        }

        return false;
    }

    class InputHolder {
        public ArrayList<StateSubsystem.SubsystemState> requestedSubsystemStates;

        Translation2d driverLeftJoystickPosition;
        Translation2d driverRightJoystickPosition;

        boolean resetNavX;
    }

    public enum ControllerSide {
        LEFT, RIGHT
    }
}
