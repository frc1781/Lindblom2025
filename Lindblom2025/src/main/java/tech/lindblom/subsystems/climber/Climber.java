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

        mRightArm.setInverted(true);
        mLeftArm.setInverted(true);

        mLeftEncoder = mLeftArm.getEncoder();
        mRightEncoder = mRightArm.getEncoder();
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                mLeftEncoder.setPosition(1/4); //SUBJECT TO CHANGE
                mRightEncoder.setPosition(1/4); //SUBJECT TO CHANGE

                mLeftArm.set(0);
                mRightArm.set(0);
                break;
            case WAIT:
                mLeftEncoder.setPosition(0); //SUBJECT TO CHANGE
                mRightEncoder.setPosition(0); //SUBJECT TO CHANGE

                mLeftArm.set(0);
                mRightArm.set(0);
                break;
            case LIFT:
                mLeftEncoder.setPosition(0); //SUBJECT TO CHANGE(Unit is rotations)
                mRightEncoder.setPosition(0); //SUBJECT TO CHANGE(Unit is rotations)

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

        double mLeftMotorVelocity = mLeftEncoder.getVelocity();  //unit is ROTATIONS per MINUTE(NOT SECONDS)
        double mRightMotorVelocity = mRightEncoder.getVelocity();  //unit is ROTATIONS per MINUTE(NOT SECONDS)

        double leftMotorDutyCycle = 0;
        double rightMotorDutyCycle = 0;



        switch((ClimberState)getCurrentState()) {
            case IDLE:


                break;
            case WAIT:
                leftMotorDutyCycle = -0.1;
                rightMotorDutyCycle = -0.1;
                break;
            case LIFT:
                leftMotorDutyCycle = 0.1;
                rightMotorDutyCycle = 0.1;
                break;
            default:
                leftMotorDutyCycle = 0;
                rightMotorDutyCycle = 0;
                break;
        }

        if (Math.abs(mLeftMotorVelocity) > 20) {    //Subject to change
            //slow down left motor by 3/4ths (idk how much more)
            leftMotorDutyCycle *= 1/4;
        }

        if (Math.abs(mRightMotorVelocity) > 20) {     //Subject to change
            //slow down left motor by 3/4ths (idk how much more)
            rightMotorDutyCycle *= 1/4;
        }

        mLeftArm.set(leftMotorDutyCycle);
        mRightArm.set(rightMotorDutyCycle);
    }

    public enum ClimberState implements SubsystemState{
        IDLE, WAIT, LIFT;
    }
    
}
