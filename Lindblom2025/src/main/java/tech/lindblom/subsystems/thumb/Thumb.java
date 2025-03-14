package tech.lindblom.subsystems.thumb;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public class Thumb extends StateSubsystem {
    private final SparkFlex spinMotor;

    public Thumb() {
        super("Thumb", ThumbState.IDLE);

        //Spin Motor
        spinMotor = new SparkFlex(Constants.Thumb.SPIN_THUMB_MOTOR, MotorType.kBrushless);
        SparkFlexConfig spinMotorConfig = new SparkFlexConfig();
        spinMotorConfig.idleMode(IdleMode.kCoast);
        spinMotorConfig.smartCurrentLimit(30);
        spinMotorConfig.inverted(false);
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        switch ((ThumbState) getCurrentState()) {
            case SPIN_IN:
                spinMotor.set(-0.75);
                break;
            case SPIN_OUT:
                spinMotor.set(0.5);
                break;
            case IDLE:
                spinMotor.set(0);
                break;
        }
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ThumbState.IDLE;
    }

    @Override
    public void init() {
        spinMotor.set(0);
    }

    public enum ThumbState implements SubsystemState {
        SPIN_IN, SPIN_OUT, IDLE
    }
}