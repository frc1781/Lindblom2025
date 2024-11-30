package tech.lindblom.subsystems.auto.groups;

import tech.lindblom.subsystems.auto.AutoStep;

/*
    @Description Runs in order, but depends on the previous AutoStep running with matching states.
*/
public class DependGroup extends AutoStepGroup {
    public DependGroup(AutoStep[] autoSteps) {
        super(autoSteps, GroupType.DEPEND);
    }
}
