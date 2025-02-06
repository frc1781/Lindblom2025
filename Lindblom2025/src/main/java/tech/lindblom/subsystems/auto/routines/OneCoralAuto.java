package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class OneCoralAuto implements AutoRoutine {
    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[] {
                    new AutoStep(4, Auto.getPathFromName("collectCoral") ),
                    new AutoStep(3, Auto.getPathFromName("reef")),
                })
        };
    }

    @Override
    public String getName() {
        return "OneCoralAuto";
    }
}
