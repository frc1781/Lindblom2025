package tech.lindblom.subsystems.autosystem;

import tech.lindblom.subsystems.Subsystem;

public class AutoSystem extends Subsystem {
    AutoSystem(String name, SubsystemStates defaultState) {
        super(name, defaultState);
    }

    @Override
    public void init() {

    }

    @Override
    public void getToState() {
        switch (currentMode) {
            case DISABLED:

                break;
            case AUTONOMOUS:
            
                break;
                default:
                    break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        return false;
    }

    public void cacheAutoRoutine() {

    }
}
