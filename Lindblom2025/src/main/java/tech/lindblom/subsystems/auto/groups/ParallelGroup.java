package tech.lindblom.subsystems.auto.groups;

import tech.lindblom.subsystems.auto.AutoStep;

/*
    @Description Runs in order with the previous AutoStep.
*/

public class ParallelGroup extends AutoStepGroup{
    public ParallelGroup(AutoStep[] autoSteps) {
        super(autoSteps, GroupType.PARALLEL);
    }
}
