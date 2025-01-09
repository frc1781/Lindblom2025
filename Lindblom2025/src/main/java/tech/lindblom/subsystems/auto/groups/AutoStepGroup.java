package tech.lindblom.subsystems.auto.groups;

import tech.lindblom.subsystems.auto.AutoStep;

public abstract class AutoStepGroup {
    AutoStep[] autoSteps = null;
    GroupType groupType;

    public AutoStepGroup(AutoStep[] autoSteps, GroupType groupType) {
        this.groupType = groupType;
        this.autoSteps = autoSteps;
    }

    public AutoStep[] getAutoSteps() {
        return autoSteps;
    }

    public GroupType getGroupType() {
        return groupType;
    }

    public enum GroupType {
        DEPEND, PARALLEL
    }
}
