package tech.lindblom.subsystems.auto;

import tech.lindblom.subsystems.auto.groups.AutoStepGroup;

public interface AutoRoutine {

    public String getName();

    public AutoStepGroup[] getAutoStepGroups();
}