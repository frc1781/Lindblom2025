package tech.lindblom.subsystems.climber;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public abstract class BaseClimber extends StateSubsystem {
    protected SparkMax leverMotor;
    protected ArmFeedforward armFeedforward;

    protected BaseClimber() {
        super("Climber", Climber.ClimberState.IDLE);
        leverMotor = new SparkMax(Constants.Climber.CLIMBER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig leverMotorConfig = new SparkMaxConfig();
        leverMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        leverMotorConfig.inverted(true);
        leverMotorConfig.encoder.positionConversionFactor(Constants.Climber.RADIANS_PER_REVOLUTION);
        leverMotorConfig.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);

        leverMotor.configure(leverMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);
    }

    @Override
    public abstract boolean matchesState();

    @Override
    public abstract void init();

    @Override
    public abstract void periodic();

    public abstract double getMotorVelocity();


    public enum ClimberState implements SubsystemState{
        IDLE, DOWN, UP, HOLD;
    }
}
