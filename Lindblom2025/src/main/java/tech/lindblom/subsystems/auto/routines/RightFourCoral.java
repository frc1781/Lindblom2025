package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class RightFourCoral implements AutoRoutine {
    @Override
    public String getName() {
        return "Right Four Coral";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(
                        new AutoStep[] {
                                new AutoStep(RobotController.Action.START_ARM),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("start;EF")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT, Auto.getPathFromName("EF;collect")),
                                new AutoStep(RobotController.Action.CENTER_REEF_RIGHT_L4, Auto.getPathFromName("collect;CD")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT, Auto.getPathFromName("CD;collect")),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("collect;CD")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT, Auto.getPathFromName("CD;collect")),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("collectNorth;AB"))
                        }
                )
        };
    }
}
