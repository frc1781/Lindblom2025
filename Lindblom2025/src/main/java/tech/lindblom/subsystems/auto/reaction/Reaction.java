package tech.lindblom.subsystems.auto.reaction;

import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;

import java.util.ArrayList;
import java.util.List;

public interface Reaction {
    public AutoStep[] getReaction(ArrayList<StateSubsystem> failedSubsystems);
}
