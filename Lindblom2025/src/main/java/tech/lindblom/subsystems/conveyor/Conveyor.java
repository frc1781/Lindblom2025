package tech.lindblom.subsystems.conveyor;

import tech.lindblom.subsystems.types.StateSubsystem;

public class Conveyor extends StateSubsystem {

    

    public Conveyor() {
        super("conveyor", ConveyorState.IDLE);        
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

    public enum ConveyorState implements SubsystemState {
        IDLE        
    }
    
}
