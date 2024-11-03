package tech.lindblom.subsystems.auto.routines;

import tech.lindblom.control.ControlSystem;
import tech.lindblom.subsystems.auto.AutoRoutine;
import tech.lindblom.subsystems.auto.AutoStep;

public class TestRoutine implements AutoRoutine {
    @Override
    public String getName() {
        return "Movement Test";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(ControlSystem.Action.WAIT)
        };
    }
}
