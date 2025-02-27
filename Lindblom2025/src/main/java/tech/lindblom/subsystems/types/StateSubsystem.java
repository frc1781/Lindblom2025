package tech.lindblom.subsystems.types;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.EnumCollection;
import edu.wpi.first.wpilibj.Timer;

public abstract class StateSubsystem extends Subsystem {
    private SubsystemState currentState;
    public final SubsystemState defaultState;
    protected Timer timeInState;
    
    protected StateSubsystem(String _name, SubsystemState _defaultState) {
        super(_name);
        defaultState = _defaultState;
        currentState = defaultState;
        timeInState = new Timer();
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public abstract boolean matchesState();

    private void resetTimeInState() {
        timeInState.reset();
        timeInState.start();
    };

    public void setState(SubsystemState newState) {
        if (currentState == newState) {
            return;
        }
        System.out.println(name + " changed to " + newState.toString());
        stateTransition(currentState, newState);
        currentState = newState;
        resetTimeInState();
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public void stateTransition(SubsystemState previousState, SubsystemState newState) {}

    public SubsystemState getCurrentState() {
        return currentState;
    }
    public void restoreToDefaultState() {
        setState(defaultState);
    }

    public interface SubsystemState {}
}
