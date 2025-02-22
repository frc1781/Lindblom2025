package tech.lindblom.subsystems.auto.routines;

import frc.robot.Robot;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.arm.Arm.ArmState;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class ThreeCoralAuto implements AutoRoutine {
    @Override
    public String getName() {
        return "Three Coral Auto";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[] {
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("start;EF")),
                        new AutoStep(RobotController.Action.L4),
                        new AutoStep(RobotController.Action.COLLECT, Auto.getPathFromName("EF;collect")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("collect;CD")),
                        new AutoStep(RobotController.Action.L4),
                        new AutoStep(RobotController.Action.COLLECT, Auto.getPathFromName("CD;collect")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("collect;CD")),
                        new AutoStep(RobotController.Action.L3)
                })
        };
    }
}
