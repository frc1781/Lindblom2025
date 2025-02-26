package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.reaction.Reaction;

public class AutoStep {
    private final StepType stepType;

    private int maxTime;
    private RobotController.Action action;
    private PathPlannerPath path = null;
    private Reaction reaction;

    public AutoStep(int maxTime, PathPlannerPath path) {
        this.maxTime = maxTime;
        this.path = path;

        this.stepType = StepType.PATH;
    }

    public AutoStep(int maxTime, RobotController.Action action) {
        this.maxTime = maxTime;
        this.action = action;

        this.stepType = StepType.ACTION;
    }

    public AutoStep(int maxTime, RobotController.Action action, PathPlannerPath path) {
        this.maxTime = maxTime;
        this.action = action;
        this.path = path;

        this.stepType = StepType.PATH_AND_ACTION;
    }

    public AutoStep(int maxTime, PathPlannerPath path, Reaction reaction) {
        this.maxTime = maxTime;
        this.path = path;
        this.reaction = reaction;

        this.stepType = StepType.PATH;
    }

    public AutoStep(int maxTime, RobotController.Action action, Reaction reaction) {
        this.maxTime = maxTime;
        this.action = action;
        this.reaction = reaction;

        this.stepType = StepType.ACTION;
    }

    public AutoStep(int maxTime, RobotController.Action action, PathPlannerPath path, Reaction reaction) {
        this.maxTime = maxTime;
        this.action = action;
        this.path = path;
        this.reaction = reaction;

        this.stepType = StepType.PATH_AND_ACTION;
    }

    public AutoStep(PathPlannerPath path) {
        this.path = path;

        this.stepType = StepType.PATH;
    }

    public AutoStep(RobotController.Action action) {
        this.action = action;

        this.stepType = StepType.ACTION;
    }

    public AutoStep(RobotController.Action action, PathPlannerPath path) {
        this.action = action;
        this.path = path;

        this.stepType = StepType.PATH_AND_ACTION;
    }

    public boolean hasReaction() {
        return reaction != null;
    }

    public Reaction getReaction() {
        return reaction;
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

    public enum StepType {
        ACTION,
        PATH,
        PATH_AND_ACTION,
    }
}
