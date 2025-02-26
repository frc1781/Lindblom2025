package tech.lindblom.subsystems.conveyor;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import tech.lindblom.subsystems.types.StateSubsystem;

public abstract class BaseConveyor extends StateSubsystem {
    protected DigitalInput coralHopperSensorFront;
    protected DigitalInput coralHopperSensorBack;
    protected DigitalInput coralCradleSensor;
    protected DigitalInput sideRampSensor;
    protected DigitalInput backRampSensor;

    protected SparkMax coralConveyor;

    protected BaseConveyor(String _name, SubsystemState _defaultState) {
        super(_name, _defaultState);

    }

    @Override
    public boolean matchesState() {
        return false;
    }

    @Override
    public void init() {

    }

    public enum ConveyorState implements SubsystemState {
        IDLE, CONVEY, COLLECT
    }
}
