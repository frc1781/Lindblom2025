package tech.lindblom.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public class Climber extends StateSubsystem{

    public final SparkMax mArm;
    public final RelativeEncoder mArmEncoder;

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mArm = new SparkMax(Constants.Climber.RIGHT_ARM, SparkLowLevel.MotorType.kBrushless);

        mArm.setInverted(true);

        mArmEncoder = mArm.getEncoder();
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        mArm.set(0);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                mArmEncoder.setPosition(0); //SUBJECT TO CHANGE
                break;
            case LIFT:
                mArmEncoder.setPosition(0); //SUBJECT TO CHANGE(Unit is rotations)
                break;
        }
    }

    @Override
    public void periodic() {
        Rotation2d mMotorPosition = Rotation2d.fromRotations(mArmEncoder.getPosition());
        double mMotorVelocity = mArmEncoder.getVelocity();  //unit is ROTATIONS per MINUTE(NOT SECONDS)
        double motorDutyCycle = 0;

        switch((ClimberState)getCurrentState()) {
            case IDLE:

                break;
            case LIFT:
                motorDutyCycle = 0.1;
                break;
        }

        mArm.set(motorDutyCycle);
    }

    public enum ClimberState implements SubsystemState{
        IDLE, LIFT;
    }
    
}
