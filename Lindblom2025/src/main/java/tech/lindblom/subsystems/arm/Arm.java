package tech.lindblom.subsystems.arm;

import tech.lindblom.subsystems.types.StateSubsystem;

public class Arm extends StateSubsystem {

    public Arm() {
        super("Arm", ArmState.IDLE);
    }

    @Override
    public boolean matchesState() {
        return false;
    }

    @Override
    public void init() {
        
    }

    @Override
    public void periodic() {
        
    }

    public enum ArmState implements SubsystemState {
        IDLE
    }
    
}
