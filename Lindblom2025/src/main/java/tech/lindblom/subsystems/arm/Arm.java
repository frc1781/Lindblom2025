package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public class Arm extends StateSubsystem {

    private SparkMax armMotor;

    public Arm() {
        super("Arm", ArmState.IDLE);
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(30);
        armMotorConfig.encoder.positionConversionFactor((1/4) * (1/5) * (1/2));
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
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
