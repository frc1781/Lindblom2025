package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class CenterOneCoral implements AutoRoutine {
    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[] {
                        new AutoStep(RobotController.Action.SCORE_START_ARM, Auto.getPathFromName("start;HG")),
                        new AutoStep(RobotController.Action.CENTER_REEF_RIGHT_L4, 10, 21),
                })
        };
    }

    @Override
    public String getName() {
        return "Center One Coral";
    }
}
