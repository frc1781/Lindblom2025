package tech.lindblom.subsystems.types;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.EnumCollection;
import edu.wpi.first.wpilibj.Timer;

public abstract class StateSubsystem extends Subsystem {
    private SubsystemState currentState;
    public final SubsystemState defaultState;
    protected EnumCollection.OperatingMode currentOperatingMode;
    protected Timer timeInState;
    
    protected StateSubsystem(String _name, SubsystemState _defaultState) {
        super(_name);
        defaultState = _defaultState;
        currentState = defaultState;
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public void setOperatingMode(EnumCollection.OperatingMode mode) {
        currentOperatingMode = mode;
        System.out.println(name + " initialized into operating mode " + mode.toString());
        init();
    }

    public abstract boolean matchesState();

    private void changedState() {  
        timeInState.reset();
        timeInState.start();
    };

    public void setState(SubsystemState newState) {
        if (currentState == newState) {
            return;
        }
        System.out.println(name + " changed to " + newState.toString());
        currentState = newState;
        changedState();
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }
    public SubsystemState getCurrentState() {
        return currentState;
    }
    public void restoreToDefaultState() {
        setState(defaultState);
    }

    public void periodic() {
        Logger.recordOutput(name + "/timeInCurrentState", timeInState.get());
    }

    public interface SubsystemState {}
}
