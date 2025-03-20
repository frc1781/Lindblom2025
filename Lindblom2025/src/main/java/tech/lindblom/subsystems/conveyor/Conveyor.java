package tech.lindblom.subsystems.conveyor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Conveyor extends StateSubsystem {
    RobotController controller;
    protected DigitalInput coralHopperSensorFront;
    protected DigitalInput coralHopperSensorBack;
    protected DigitalInput coralCradleSensor;
    protected DigitalInput sideRampSensor;
    protected DigitalInput backRampSensor;

    protected SparkMax coralConveyor;

    public Conveyor(RobotController controller) {
        super("Conveyor", ConveyorState.IDLE);
        this.controller = controller;
        // If .get() is false the beam was broken - Returns false if the circuit is open.
        coralHopperSensorFront = new DigitalInput(Constants.Conveyor.CORAL_HOPPER_SENSOR_FRONT_DIO);
        coralHopperSensorBack = new DigitalInput(Constants.Conveyor.CORAL_HOPPER_SENSOR_BACK_DIO);
        coralCradleSensor = new DigitalInput(Constants.Conveyor.CORAL_CRADLE_SENSOR_DIO);
        sideRampSensor = new DigitalInput(Constants.Conveyor.SIDE_RAMP_DIO);
        backRampSensor = new DigitalInput(Constants.Conveyor.BACK_RAMP_DIO);

        coralConveyor = new SparkMax(Constants.Conveyor.CORAL_CONVEYOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig coralConveyorConfig = new SparkMaxConfig();
        coralConveyorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        coralConveyorConfig.smartCurrentLimit(30);
        coralConveyor.configure(coralConveyorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public boolean matchesState() {
        return switch ((ConveyorState) getCurrentState()) {
            case IDLE:
                yield false;
            case COLLECT:
                yield conveyorHasCoral();
            case CONVEY:
                yield cradleHasCoral();
        };
    }

    @Override
    public void init() {
        super.init();
        coralConveyor.set(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/coralHopperSensorFront", coralHopperSensorFront.get());
        Logger.recordOutput(this.name + "/coralHopperSensorBack", coralHopperSensorBack.get());
        Logger.recordOutput(this.name + "/sideRampSensor", sideRampSensor.get());
        Logger.recordOutput(this.name + "/backRampSensor", backRampSensor.get());
        Logger.recordOutput(this.name + "/coralCradleSensor", coralCradleSensor.get());

        if (currentOperatingMode == EnumCollection.OperatingMode.DISABLED) return;
        if (conveyorHasCoral() && getCurrentState() == ConveyorState.IDLE) {
            setState(ConveyorState.CONVEY);
        }

        switch ((ConveyorState) getCurrentState()) {
            case IDLE:
                coralConveyor.set(0);
                break;
            case CONVEY:
                    if (!controller.elevatorInConveyPosition()) 
                        return;
                    if ((cradleHasCoral() && !conveyorHasCoral())) {
                        setState(ConveyorState.IDLE);
                    }
                    if (!cradleHasCoral() && !conveyorHasCoral()) {
                        setState(ConveyorState.IDLE);
                    }

                    if (timeInState.get() < 0.5) {
                        return;
                    }

                    coralConveyor.set(.5);
                break;
        }
    }

    public boolean conveyorHasCoral() {
        return !sideRampSensor.get() || !backRampSensor.get() || !coralHopperSensorFront.get() || !coralHopperSensorBack.get();
    }

    public boolean cradleHasCoral() {
        return !coralCradleSensor.get();
    }

    public boolean testCoralCollectAndConvey() {
        return !sideRampSensor.get() || !backRampSensor.get() || !coralHopperSensorFront.get() || !coralHopperSensorBack.get() || !coralCradleSensor.get();
    }

    public enum ConveyorState implements SubsystemState {
        IDLE, CONVEY, COLLECT
    }
}
