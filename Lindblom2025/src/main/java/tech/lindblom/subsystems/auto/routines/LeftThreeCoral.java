package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController.Action;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class LeftThreeCoral implements AutoRoutine {

    @Override
    public String getName() {
        return "Left Three Coral";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(
                    new AutoStep[] {
                        new AutoStep(Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("start;IJ"), 11, 20),
                        new AutoStep(Action.READY_FOR_COLLECT, Auto.getPathFromName("IJ;collect")),
                        new AutoStep(Action.CONVEY_AND_COLLECT),
                        new AutoStep(Action.CENTER_REEF_RIGHT_L4, Auto.getPathFromName("collect;LK"), 6, 19),
                        new AutoStep(Action.READY_FOR_COLLECT, Auto.getPathFromName("LK;collect")),
                        new AutoStep(Action.CONVEY_AND_COLLECT),
                        new AutoStep(Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("collect;LK"), 6, 19),
                    }
                )
        };
    }
}
