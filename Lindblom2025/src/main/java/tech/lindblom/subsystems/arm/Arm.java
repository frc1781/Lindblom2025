package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.HashMap;

import com.revrobotics.spark.SparkLowLevel;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection.OperatingMode;

public class Arm extends StateSubsystem {

    private SparkMax armMotor;
    private HashMap<ArmState,Double> positionMap;

    public Arm() {
        super("Arm", ArmState.IDLE);

        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(30);
        armMotorConfig.encoder.positionConversionFactor(360.0/(4.0 * 5.0 * 2.0));
        armMotorConfig.closedLoop.pid(0.001, 0,0);
        armMotorConfig.openLoopRampRate(5.0);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positionMap = new HashMap<>();
        positionMap.put(ArmState.IDLE, 0.0);
        positionMap.put(ArmState.L1, 0.0);
        positionMap.put(ArmState.L2, 0.0);
        positionMap.put(ArmState.L3, 0.0);
        positionMap.put(ArmState.L4, 0.0);
    }

    @Override
    public boolean matchesState() {
       double tolerance = 2;
       return Math.abs(positionMap.get(getCurrentState()) - armMotor.getEncoder().getPosition()) <= tolerance;
       
    }

    @Override
    public void init() {
       
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/MotorEncoder", armMotor.getEncoder().getPosition());
        if(currentMode == OperatingMode.DISABLED) return;
        switch ((ArmState) getCurrentState()) {
            case IDLE:
                armMotor.set(0);
                break;
            case MANUAL_DOWN:
                armMotor.set(-0.05);
                break;
            case MANUAL_UP:
                armMotor.set(0.05);
                break;

        }
        getToPosition(positionMap.get(getCurrentState()));
    }

    private void getToPosition(double position ){
        //armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    public enum ArmState implements SubsystemState {
        IDLE, L1, L2, L3, L4, MANUAL_UP, MANUAL_DOWN
    }
}
