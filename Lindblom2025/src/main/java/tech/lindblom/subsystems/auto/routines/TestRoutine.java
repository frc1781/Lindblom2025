package tech.lindblom.subsystems.auto.routines;

import com.pathplanner.lib.path.PathPlannerPath;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.groups.DependGroup;

public class TestRoutine implements AutoRoutine {
    @Override
    public String getName() {
        return "Movement Test";
    }

    @Override
    public AutoStepGroup[] getAutoStepGroups() {
        return new AutoStepGroup[] {
                new DependGroup(new AutoStep[]{
                        new AutoStep(10, Auto.getPathFromName("testPath"))
                })
        };
    }
}
