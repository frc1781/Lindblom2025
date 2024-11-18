package tech.lindblom.subsystems.types;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.EnumCollection;

public abstract class StateSubsystem {
    protected final String name;
    SubsystemState currentState;
    public final SubsystemState defaultState;
    protected EnumCollection.OperatingMode currentOperatingMode;

    protected StateSubsystem(String _name, SubsystemState _defaultState) {
        name = _name;
        defaultState = _defaultState;
        currentState = defaultState;
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public void setOperatingMode(EnumCollection.OperatingMode mode) {
        currentOperatingMode = mode;
        System.out.println(name + " initialized into operating mode " + mode.toString());
        init();
    }

    public abstract void init();
    public abstract void getToState();
    public abstract boolean matchesState();
    public void setState(SubsystemState newState) {
        if (currentState == newState) {
            return;
        }
        currentState = newState;
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }
    public SubsystemState getCurrentState() {
        return currentState;
    }
    public void setDefaultState() {
        setState(defaultState);
    }

    public interface SubsystemState {}
}
