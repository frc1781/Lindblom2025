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
                                new AutoStep(RobotController.Action.START_ARM, Auto.getPathFromName("startRight")),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, 9, 22),

                                new AutoStep(RobotController.Action.READY_FOR_COLLECT, Auto.getPathFromName("EF;collect")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT),
                                new AutoStep(RobotController.Action.READY_FOP_POLE, Auto.getPathFromName("collect;CD")),
                                new AutoStep(RobotController.Action.CENTER_REEF_RIGHT_L4, 8, 17),

                                new AutoStep(RobotController.Action.READY_FOR_COLLECT, Auto.getPathFromName("EF;collect")),
                                new AutoStep(RobotController.Action.CONVEY_AND_COLLECT),
                                new AutoStep(RobotController.Action.READY_FOP_POLE, Auto.getPathFromName("collect;CD")),
                                new AutoStep(RobotController.Action.CENTER_REEF_LEFT_L4, 8, 17),
                        }
                )
        };
    }
}
