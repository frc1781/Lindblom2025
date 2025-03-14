package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class RightThreeCoral implements AutoRoutine {
    @Override
    public String getName() {
        return "Right Three Coral";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[]{
                new DependGroup(
                        new AutoStep[] {
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("start;EF")),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, 9, 22),
                                new AutoStep(RobotController.Action.READY_FOR_COLLECT, Auto.getPathFromName("EF;collect")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT),
                                new AutoStep(RobotController.Action.CENTER_REEF_RIGHT_L4, Auto.getPathFromName("collect;CD"), 8, 17),
                                new AutoStep(RobotController.Action.READY_FOR_COLLECT, Auto.getPathFromName("CD;collect")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("collect;CD"), 8, 17),
                        }
                )
        };
    }
}
