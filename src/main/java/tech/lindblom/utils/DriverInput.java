package tech.lindblom.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;

public class DriverInput {

    private GenericHID[] mControllers = new GenericHID[] {
            new XboxController(0)
        };

    private LinkedHashMap<String, Event> mClickEvents = new LinkedHashMap<>();
    private LinkedHashMap<String, Event> mHoldEvents = new LinkedHashMap<>();

    private HashMap<String, Boolean> mButtonMap = new HashMap<>();
    private HashSet<String> mPressedButtons = new HashSet<>();

    public enum ControllerSide {
        LEFT, RIGHT
    }

    public DriverInput run() {
        updatePressedButtons();
        checkClickEvents();
        checkHoldEvents();
        return this;
    }

    public EVector getControllerJoyAxis(ControllerSide side, int controllerIndex) {
        var selectedController = (XboxController) mControllers[controllerIndex];
        EVector ret_val = new EVector();
        if (side == ControllerSide.LEFT) {
            ret_val.x = selectedController.getLeftX();
            ret_val.y = selectedController.getLeftY();
        } else {
            ret_val.x = selectedController.getRightX();
            ret_val.y = selectedController.getRightY();
        }

        return ret_val;
    }

    public EVector getJoystickJoyAxis(int joyIndex) {

        var selectedJoystick = (Joystick) mControllers[1];
        EVector ret_val = new EVector();

        ret_val.x = selectedJoystick.getX();
        ret_val.y = selectedJoystick.getY();

        return ret_val;
    }

    public void addClickListener(int controllerPort, String button, Event event) {
        String key = createKey(controllerPort, button);

        mClickEvents.put(key, event);
    }

    public void addClickListener(int joyPort, int button, Event event) {
        String key = createKey(joyPort, button);

        mClickEvents.put(key, event);
    }

    public void addHoldListener(int controllerPort, String button, Event event) {
        String key = createKey(controllerPort, button);

        mHoldEvents.put(key, event);
    }

    public void addHoldListener(int joyPort, int button, Event event) {
        String key = createKey(joyPort, button);

        mHoldEvents.put(key, event);
    }

    public boolean getButton(int controllerPort, String button) {
        String key = createKey(controllerPort, button);

        return mButtonMap.get(key);
    }

    public boolean getButton(int joyPort, int button) {
        String key = createKey(joyPort, button);

        return mButtonMap.get(key);
    }

    public EVector getTriggerAxis(int controllerPort) {
        XboxController controller = (XboxController) mControllers[controllerPort];

        return EVector.newVector(controller.getLeftTriggerAxis(), controller.getRightTriggerAxis());
    }

    private void checkClickEvents() {
        for (String key : mClickEvents.keySet()) {
            boolean buttonPressed = mButtonMap.get(key);

            if (buttonPressed && !mPressedButtons.contains(key)) {
                mClickEvents.get(key).onPress(true);
                mPressedButtons.add(key);
            } else if (!buttonPressed && mPressedButtons.contains(key)) {
                mPressedButtons.remove(key);
            }

            // System.out.printf("%s :: %b %n", key, buttonPressed);

        }
    }

    private void checkHoldEvents() {
        for (String key : mHoldEvents.keySet()) {
            boolean buttonPressed = mButtonMap.get(key);

            mHoldEvents.get(key).onPress(buttonPressed);
        }
    }

    public int getPOV(int controllerPort) {
        return ((XboxController) mControllers[controllerPort]).getPOV();
    }

    private void updatePressedButtons() {
        for (int i = 0; i < mControllers.length; i++) {
            GenericHID selectedController = mControllers[i];

            if (selectedController instanceof XboxController) {
                updateControllerButtons(i, (XboxController) selectedController);
            } else {
                updateJoystickButtons(i, (Joystick) selectedController);
            }

        }
    }

    private void updateControllerButtons(int index, XboxController controller) {
        mButtonMap.put(createKey(index, "LB"), controller.getLeftBumper());
        mButtonMap.put(createKey(index, "RB"), controller.getRightBumper());
        mButtonMap.put(createKey(index, "Y"), controller.getYButton());
        mButtonMap.put(createKey(index, "X"), controller.getXButton());
        mButtonMap.put(createKey(index, "B"), controller.getBButton());
        mButtonMap.put(createKey(index, "A"), controller.getAButton());
        mButtonMap.put(createKey(index, "START"), controller.getStartButton());
        mButtonMap.put(createKey(index, "BACK"), controller.getBackButton());
        mButtonMap.put(createKey(index, "N"), controller.getPOV() == 0);
        mButtonMap.put(createKey(index, "NE"), controller.getPOV() == 45);
        mButtonMap.put(createKey(index, "E"), controller.getPOV() == 90);
        mButtonMap.put(createKey(index, "SE"), controller.getPOV() == 135);
        mButtonMap.put(createKey(index, "S"), controller.getPOV() == 180);
        mButtonMap.put(createKey(index, "SW"), controller.getPOV() == 225);
        mButtonMap.put(createKey(index, "W"), controller.getPOV() == 270);
        mButtonMap.put(createKey(index, "NW"), controller.getPOV() == 315);
        mButtonMap.put(createKey(index, "R3"), controller.getRightStickButton());
        mButtonMap.put(createKey(index, "L3"), controller.getLeftStickButton());
        mButtonMap.put(createKey(index, "LT"), controller.getLeftTriggerAxis() >= 0.9);
        mButtonMap.put(createKey(index, "RT"), controller.getRightTriggerAxis() >= 0.9);
    }

    private void updateJoystickButtons(int index, Joystick joystick) {
        for (int i = 1; i < 13; i++) {
            mButtonMap.put(createKey(index, i), joystick.getRawButton(i));
        }
    }

    // {type}:{port}:{button}
    private String createKey(int controllerPort, String button) {
        StringBuilder builder = new StringBuilder();

        builder.append("controller");
        builder.append(controllerPort);
        builder.append(button);

        return builder.toString();
    }

    // {type}:{port}:{button}
    private String createKey(int joyPort, int button) {
        StringBuilder builder = new StringBuilder();

        builder.append("joystick");
        builder.append(joyPort);
        builder.append(button);

        return builder.toString();
    }

    public interface Event {
        public void onPress(boolean isPressed);
    }

    public void addButtonListener(int coPilotPort, String string, Object object) {
        throw new UnsupportedOperationException("Unimplemented method 'addButtonListener'");
    }

}
