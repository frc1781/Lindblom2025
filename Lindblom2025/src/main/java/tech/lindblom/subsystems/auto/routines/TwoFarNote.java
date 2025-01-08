package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class TwoFarNote implements AutoRoutine {
    @Override
    public String getName() {
        return "Two Far Note";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[]{
                        new AutoStep(10, Auto.getPathFromName("p2;c3")),
                        new AutoStep(10, Auto.getPathFromName("c3;p2")),
                }),
                new DependGroup(new AutoStep[]{
                        new AutoStep(10, Auto.getPathFromName("p2;c1")),
                        new AutoStep(10, Auto.getPathFromName("c1;p2")),
                })
        };
    }
}
