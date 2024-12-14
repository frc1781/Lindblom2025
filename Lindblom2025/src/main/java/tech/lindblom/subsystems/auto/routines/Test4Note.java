package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;

public class Test4Note implements AutoRoutine {
    @Override
    public String getName() {
        return "4 Note";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(20, RobotController.Action.SUB_SHOOT),
                new AutoStep(20, RobotController.Action.COLLECT, Auto.getPathFromName("p2;n2")),
                new AutoStep(20, RobotController.Action.TWO_NOTE),
                new AutoStep(20, RobotController.Action.COLLECT, Auto.getPathFromName("n2;n1")),
                new AutoStep(20, RobotController.Action.ONE_NOTE),
                new AutoStep(20, RobotController.Action.COLLECT, Auto.getPathFromName("n1;n3")),
                new AutoStep(20, RobotController.Action.THIRD_NOTE),
                new AutoStep(20, RobotController.Action.COLLECT, Auto.getPathFromName("n3;c1")),
                new AutoStep(20, RobotController.Action.TEST_GREEN, Auto.getPathFromName("c1;n3")),
                new AutoStep(20, RobotController.Action.THIRD_NOTE),
        };
    }
}
