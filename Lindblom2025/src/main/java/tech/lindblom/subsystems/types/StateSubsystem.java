package tech.lindblom.subsystems.types;

import tech.lindblom.utils.EnumCollection;

public abstract class StateSubsystem {
    protected final String name;
    SubsystemState currentState;
    protected final SubsystemState defaultState;
    protected EnumCollection.OperatingMode currentOperatingMode;

    protected StateSubsystem(String _name, SubsystemState _defaultState) {
        name = _name;
        defaultState = _defaultState;
        currentState = defaultState;
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
    }

    public interface SubsystemState {}
}
