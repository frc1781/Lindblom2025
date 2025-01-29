package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.HashMap;

import com.revrobotics.spark.SparkLowLevel;

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
        armMotorConfig.encoder.positionConversionFactor((1/4) * (1/5) * (1/2) * 360);
        armMotorConfig.closedLoop.pid(0.001, 0,0);
        armMotorConfig.openLoopRampRate(5);
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
        if(currentMode == OperatingMode.DISABLED) return;
        //getToPosition(positionMap.get(getCurrentState()));
    }

    private void getToPosition(double position ){
        armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    public enum ArmState implements SubsystemState {
        IDLE, L1, L2, L3, L4
    }
}
