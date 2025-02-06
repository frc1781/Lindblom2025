package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class Collect implements AutoRoutine {

    @Override
    public String getName() {
        return "Collect L4";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[] {
                        new AutoStep(RobotController.Action.COLLECT),
                        new AutoStep(RobotController.Action.L4)
                })
        };
    }
}
