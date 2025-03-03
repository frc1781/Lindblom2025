package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class LeftFourCoral implements AutoRoutine {
    @Override
    public String getName() {
        return "Left Four Coral";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(
                        new AutoStep[] {
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("start;IJ")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT, Auto.getPathFromName("IJ;collect")),
                                new AutoStep(RobotController.Action.CENTER_REEF_RIGHT_L4, Auto.getPathFromName("collect;LK")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT, Auto.getPathFromName("LK;collect")),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, Auto.getPathFromName("collect;LK")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT, Auto.getPathFromName("LK;collect")),
                                new AutoStep(RobotController.Action.CENTER_REEF_RIGHT_L4, Auto.getPathFromName("collectSouth;AB")),
                        }
                )
        };
    }
}
