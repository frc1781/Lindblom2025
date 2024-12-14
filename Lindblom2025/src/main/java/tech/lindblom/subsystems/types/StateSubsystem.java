package tech.lindblom.subsystems.types;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.utils.EnumCollection;

public abstract class StateSubsystem extends Subsystem {
    SubsystemState currentState;
    public final SubsystemState defaultState;
    protected EnumCollection.OperatingMode currentOperatingMode;
    protected RobotController robotController;

    protected StateSubsystem(String _name, SubsystemState _defaultState, RobotController robotController) {
        super(_name);
        defaultState = _defaultState;
        currentState = defaultState;
        this.robotController = robotController;
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public void setOperatingMode(EnumCollection.OperatingMode mode) {
        currentOperatingMode = mode;
        System.out.println(name + " initialized into operating mode " + mode.toString());
        init();
    }

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
    public void restoreToDefaultState() {
        setState(defaultState);
    }

    public interface SubsystemState {}
}
