package tech.lindblom.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public class Climber extends StateSubsystem{

    private final SparkMax leverMotor;
    private SparkMaxSim levelMotorSimRunner;
    private final RelativeEncoder mArmEncoder;

    private ClimberState PreviousState;
    private Rotation2d positionToHold = new Rotation2d();

    private ArmFeedforward armFeedforward;

    public Climber() {
        super("Climber", ClimberState.IDLE);
        leverMotor = new SparkMax(Constants.Climber.RIGHT_ARM, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig leverMotorConfig = new SparkMaxConfig();
        leverMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        leverMotorConfig.inverted(true);
        leverMotorConfig.encoder.positionConversionFactor(Constants.Climber.RADIANS_PER_REVOLUTION);
        leverMotorConfig.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);

        mArmEncoder = leverMotor.getEncoder();
        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);

        if (!RobotBase.isReal()) {
            DCMotor motorSim = DCMotor.getNEO(1);
            levelMotorSimRunner = new SparkMaxSim(leverMotor, motorSim);
        }
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        PreviousState = (ClimberState) this.getCurrentState();
        leverMotor.set(0);
        mArmEncoder.setPosition(0);
    }

    public double getMotorVelocity() {
        return mArmEncoder.getVelocity() / 60;
    }

    @Override
    public void periodic() {
        Rotation2d mMotorPosition = Rotation2d.fromRadians(mArmEncoder.getPosition());
        double mMotorVelocity = getMotorVelocity();

        Logger.recordOutput("Climber/Velocity", mMotorVelocity);
        Logger.recordOutput("Climber/Position", mMotorPosition);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                leverMotor.set(0);
                break;
            case LIFT:
                leverMotor.set(0.5);
                break;
            case HOLD:
                if (PreviousState != ClimberState.HOLD) {
                    positionToHold = mMotorPosition;
                }

                leverMotor.getClosedLoopController().setReference(positionToHold.getRadians(), SparkBase.ControlType.kVoltage, ClosedLoopSlot.kSlot0, armFeedforward.calculate(positionToHold.getDegrees(), (Math.PI / 2)));
                break;
        }

        PreviousState = (ClimberState)this.getCurrentState();
    }

    public enum ClimberState implements SubsystemState{
        IDLE, LIFT, HOLD;
    }
    
}
