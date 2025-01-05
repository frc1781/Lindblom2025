package tech.lindblom.subsystems.auto.routines;

import com.pathplanner.lib.path.PathPlannerPath;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;
import tech.lindblom.subsystems.led.LEDs;

public class TestRoutine implements AutoRoutine {
    @Override
    public String getName() {
        return "Movement Test";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[]{
                        new AutoStep(10, RobotController.Action.TEST_BLUE),
                        new AutoStep(10, RobotController.Action.EXPECTED_LED_FAIL),
                        new AutoStep(10, RobotController.Action.TEST_RED),
                }),
                new DependGroup(new AutoStep[]{
                        new AutoStep(10, RobotController.Action.TEST_GREEN),
                        new AutoStep(10, RobotController.Action.TEST_RED),
                        new AutoStep(10, RobotController.Action.TEST_BLUE),
                }),
                new DependGroup(new AutoStep[]{
                        new AutoStep(10, RobotController.Action.TEST_GREEN),
                        new AutoStep(10, RobotController.Action.TEST_RED),
                        new AutoStep(10, RobotController.Action.TEST_BLUE),
                })
        };
    }
}
