package tech.lindblom.subsystems.auto.reaction.reactions.TestRoutine;

import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.reaction.Reaction;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;

import java.util.ArrayList;
import java.util.List;

//not sure what the naming routine should be yet, knowing this won't change i'll make it at least somewhat sensible
public class Reaction1 implements Reaction {

    @Override
    public AutoStep[] getReaction(ArrayList<StateSubsystem> failedSubsystems) {
        return null;
    }
}
