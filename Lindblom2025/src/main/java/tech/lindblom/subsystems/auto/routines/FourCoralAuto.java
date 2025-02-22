package tech.lindblom.subsystems.auto.routines;

import frc.robot.Robot;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.arm.Arm.ArmState;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class FourCoralAuto implements AutoRoutine {
    @Override
    public String getName() {
        return "Four Coral Auto";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[] {
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("start;IJ")),
                        new AutoStep(RobotController.Action.L4),
                        new AutoStep(RobotController.Action.COLLECT, Auto.getPathFromName("IJ;collect")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("collect;LK")),
                        new AutoStep(RobotController.Action.L4),
                        new AutoStep(RobotController.Action.COLLECT, Auto.getPathFromName("LK;collect")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("collect;LK")),
                        new AutoStep(RobotController.Action.L3),
                        new AutoStep(RobotController.Action.COLLECT, Auto.getPathFromName("LK;collect")),
                        new AutoStep(RobotController.Action.CENTER_REEF_LEFT, Auto.getPathFromName("collectNorth;AB")),
                        new AutoStep(RobotController.Action.L4)
                })
        };
    }
}
