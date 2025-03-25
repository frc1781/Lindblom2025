package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.control.RobotController.Action;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class LeftOneCoral implements AutoRoutine {
    @Override
    public String getName() {
        return "Left 1 Coral";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[] {
                        new AutoStep(Action.START_ARM, Auto.getPathFromName("startLeft")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, 11, 20),
             })
        };
    }
}
