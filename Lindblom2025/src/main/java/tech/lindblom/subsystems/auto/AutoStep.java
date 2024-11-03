package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import tech.lindblom.control.ControlSystem;

public class AutoStep {
    private int maxTime;
    private ControlSystem.Action action;
    private PathPlannerPath path;
    private final boolean hasTimeLimit;

    public AutoStep(int maxTime, ControlSystem.Action action) {
        this.maxTime = maxTime;
        this.action = action;
        this.hasTimeLimit = true;
    }

    public AutoStep(int maxTime, ControlSystem.Action action, PathPlannerPath path) {
        this.maxTime = maxTime;
        this.action = action;
        this.path = path;
        this.hasTimeLimit = true;
    }

    public AutoStep(ControlSystem.Action action) {
        this.action = action;
        this.hasTimeLimit = false;
    }

    public AutoStep(ControlSystem.Action action, PathPlannerPath path) {
        this.action = action;
        this.path = path;
        this.hasTimeLimit = false;
    }

    public ControlSystem.Action getAction() {
        return action;
    }

    public int getMaxTime() {
        return maxTime;
    }
}
