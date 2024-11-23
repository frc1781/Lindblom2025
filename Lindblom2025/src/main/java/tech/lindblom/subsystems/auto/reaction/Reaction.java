package tech.lindblom.subsystems.auto.reaction;

import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.types.Subsystem;

import java.util.List;

public interface Reaction {
    public AutoStep getReaction(List<Subsystem> failedSubsystems);
}
