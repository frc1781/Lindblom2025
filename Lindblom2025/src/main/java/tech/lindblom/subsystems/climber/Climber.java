package tech.lindblom.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public class Climber extends StateSubsystem{

    public final SparkMax mLeftArm;
    public final SparkMax mRightArm;
    public final RelativeEncoder mLeftEncoder;
    public final RelativeEncoder mRightEncoder;

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftArm = new SparkMax(Constants.Climber.LEFT_ARM, SparkLowLevel.MotorType.kBrushless);
        mRightArm = new SparkMax(Constants.Climber.RIGHT_ARM, SparkLowLevel.MotorType.kBrushless);

        //mRightArm.setInverted(true);
        //mLeftArm.setInverted(true);

        mLeftEncoder = mLeftArm.getEncoder();
        mRightEncoder = mRightArm.getEncoder();
    }

    @Override
    public boolean matchesState() {
        return this.getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        switch((ClimberState)getCurrentState()) {
            case IDLE:
                mLeftArm.set(0);
                mRightArm.set(0);
                break;
            case WAIT:
                mLeftArm.set(0);
                mRightArm.set(0);
                break;
            case LIFT:
                mLeftArm.set(0);
                mRightArm.set(0);
                break;
            default:
                mLeftArm.set(0);
                mRightArm.set(0);
                break;

        }
    }

    @Override
    public void periodic() {
        Rotation2d mLeftMotorPosition = Rotation2d.fromRotations(mLeftEncoder.getPosition());
        Rotation2d mRightMotorPosition = Rotation2d.fromRotations(mRightEncoder.getPosition());

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                mLeftArm.set(0);
                mRightArm.set(0);
                break;
            case WAIT:
                mLeftArm.set(-0.5);
                mRightArm.set(-0.5);
                break;
            case LIFT:
                mLeftArm.set(.6);
                mRightArm.set(.6);
                break;
            default:
                mLeftArm.set(0);
                mRightArm.set(0);
                break;

        }
    }

    public enum ClimberState implements SubsystemState{
        IDLE, WAIT, LIFT;
    }
    
}
