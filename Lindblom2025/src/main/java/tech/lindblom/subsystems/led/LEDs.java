package tech.lindblom.subsystems.led;

import tech.lindblom.subsystems.types.StateSubsystem;

public class LEDs extends StateSubsystem {

    public LEDs() {
        super("LEDs", LEDState.BOUNCE);
    }


    @Override
    public void init() {

    }

    @Override
    public void getToState() {

    }

    @Override
    public boolean matchesState() {
        return false;
    }

    public enum LEDState implements SubsystemState{
        BOUNCE
    }
}
