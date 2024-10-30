package frc.robot.Subsystems;

import frc.robot.utils.EnumCollection;

public abstract class Subsystem {
    protected final String name;
    protected double currentTime;
    protected EnumCollection.OperatingMode currentMode;
    private final SubsystemStates defaultState;
    protected SubsystemStates currentState;

    Subsystem(String name, SubsystemStates defaultState) {
        this.defaultState = defaultState;
        this.name = name;
    }


    interface SubsystemStates {

    }
}
