package tech.lindblom.subsystems;

import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.utils.EnumCollection;
import org.littletonrobotics.junction.Logger;

public abstract class Subsystem {
    protected final String name;
    protected EnumCollection.OperatingMode currentMode;
    protected Timer autoTimer;
    private final SubsystemStates defaultState;
    protected SubsystemStates currentState;

    protected Subsystem(String name, SubsystemStates defaultState) {
        this.defaultState = defaultState;

        this.name = name;
        this.currentState = defaultState;

        this.autoTimer = new Timer();

        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public void setOperatingMode(EnumCollection.OperatingMode mode) {
        currentMode = mode;
        System.out.println(name + " initialized into operating mode " + mode.toString());
        init();
    }

    public void setCurrentState(SubsystemStates currentState) {
        this.currentState = currentState;
        Logger.recordOutput(name + "/currentState", currentState.toString());
    }

    public void getToDefaultState() {
        setCurrentState(defaultState);
    }

    public abstract void init();

    public abstract void getToState();

    public abstract boolean matchesDesiredState();

    public interface SubsystemStates {

    }
}
