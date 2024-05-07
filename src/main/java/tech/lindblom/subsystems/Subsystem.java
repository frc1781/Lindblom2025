package tech.lindblom.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class Subsystem {
    protected final String name;
    protected double currentTime;
    protected OperatingMode currentMode;
    private final SubsystemState defaultState;
    protected SubsystemState currentState;

    protected Subsystem(String _name, SubsystemState _defaultState) {
        name = _name;
        defaultState = _defaultState;
        currentState = defaultState;
    }

    public void setOperatingMode(OperatingMode mode) {
        currentMode = mode;
        System.out.println(name + " initialized into operating mode " + mode.toString());
        init();
    }

    public abstract void genericPeriodic();

    public final String getName() {
        return name;
    }

    public void setDesiredState(SubsystemState desiredState) {
        if (desiredState == currentState) {
            return;
        }

        System.out.println(desiredState);

        currentState = desiredState;
        System.out.println("Changing " + name +  "'s state to " + desiredState);
        Logger.recordOutput(name + "/DesiredState", desiredState.toString());
    }

    public final SubsystemState getState() {
        return currentState;
    }

    public final void restoreDefault() {
        setDesiredState(defaultState);
    }

    public abstract void init();

    public abstract void getToState();

    public abstract boolean matchesDesiredState();

    public abstract void autonomousPeriodic();

    public abstract void teleopPeriodic();

    public void disabledPeriodic() {

    }

    public interface SubsystemState {

    }

    public enum OperatingMode {
        DISABLED, TELEOP, AUTONOMOUS, SIMULATION, TEST
    }

}
