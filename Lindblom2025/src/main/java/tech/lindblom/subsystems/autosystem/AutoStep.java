package tech.lindblom.subsystems.autosystem;

import tech.lindblom.control.ControlSystem;

public class AutoStep {
    private int maxTime;
    private ControlSystem.Action action;

    AutoStep(int maxTime, ControlSystem.Action action) {
        this.maxTime = maxTime;
        this.action = action;
    }

/*    AutoStep(int maxTime, ControlSystem.Action action) {

    }*/

    public ControlSystem.Action getAction() {
        return action;
    }

    public int getMaxTime() {
        return maxTime;
    }
}
