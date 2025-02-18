package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
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
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("start;HG")),
                        new AutoStep(RobotController.Action.L4),
                        new AutoStep(RobotController.Action.FIND_POLE),
                        new AutoStep(Auto.getPathFromName("testPath")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("start;HG")),
                        new AutoStep(RobotController.Action.L4)
                })
        };
    }

    @Override
    public String getName() {
        return "OneCoralAuto";
    }
}
