package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import tech.lindblom.control.RobotController;

public class AutoStep {
    private int maxTime;
    private RobotController.Action action;
    private PathPlannerPath path;
    private final boolean hasTimeLimit;
    private final StepType stepType;

    public AutoStep(int maxTime, RobotController.Action action) {
        this.maxTime = maxTime;
        this.action = action;
        this.hasTimeLimit = true;

        this.stepType = StepType.ACTION;
    }

    public AutoStep(int maxTime, RobotController.Action action, PathPlannerPath path) {
        this.maxTime = maxTime;
        this.action = action;
        this.path = path;
        this.hasTimeLimit = true;

        this.stepType = StepType.PATH_AND_ACTION;
    }

    public AutoStep(RobotController.Action action) {
        this.action = action;
        this.hasTimeLimit = false;

        this.stepType = StepType.ACTION;
    }

    public AutoStep(RobotController.Action action, PathPlannerPath path) {
        this.action = action;
        this.path = path;
        this.hasTimeLimit = false;

        this.stepType = StepType.PATH;
    }

    public RobotController.Action getAction() {
        return action;
    }

    public int getMaxTime() {
        return maxTime;
    }

    public StepType getStepType() {
        return stepType;
    }

    public PathPlannerPath getPath() {
        return path;
    }

    public boolean hasTimeLimit() {
        return hasTimeLimit;
    }

    public enum StepType {
        ACTION,
        PATH,
        PATH_AND_ACTION,
    }
}
