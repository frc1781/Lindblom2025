package tech.lindblom.subsystems.auto.groups;

import tech.lindblom.subsystems.auto.AutoStep;

public class ParallelGroup extends AutoStepGroup{
    public ParallelGroup(AutoStep[] autoSteps) {
        super(autoSteps, GroupType.PARALLEL);
    }
}
