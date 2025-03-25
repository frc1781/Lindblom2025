package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.control.RobotController.Action;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class LeftThreeCoral implements AutoRoutine {

    @Override
    public String getName() {
        return "GG";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(
                    new AutoStep[] {
                        new AutoStep(Action.START_ARM, Auto.getPathFromName("gggggg")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, 11, 20),

                        new AutoStep(Action.READY_FOR_COLLECT, Auto.getPathFromName("LK;collect")),
                        new AutoStep(Action.CONVEY_AND_COLLECT),
                        new AutoStep(Action.READY_FOP_POLE, Auto.getPathFromName("collect;LK")),
                        new AutoStep(Action.CENTER_REEF_RIGHT_L4, 6, 19),

                        new AutoStep(Action.READY_FOR_COLLECT, Auto.getPathFromName("LK;collect")),
                        new AutoStep(Action.CONVEY_AND_COLLECT),
                        new AutoStep(Action.READY_FOP_POLE, Auto.getPathFromName("collect;LK")),
                        new AutoStep(Action.CENTER_REEF_LEFT_L4, 6, 19),
                    }
                )
        };
    }
}
