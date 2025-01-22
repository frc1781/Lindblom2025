package tech.lindblom.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

public class Climber extends StateSubsystem{

    public final SparkMax mArm;
    public final RelativeEncoder mArmEncoder;
    public final ProfiledPIDController mArmProfiledPID;

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mArm = new SparkMax(Constants.Climber.RIGHT_ARM, SparkLowLevel.MotorType.kBrushless);

        mArmEncoder = mArm.getEncoder();

        mArmProfiledPID = new ProfiledPIDController(0, 0, 0, null);
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        mArm.set(0);
        mArmProfiledPID.reset(0);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                mArmEncoder.setPosition(0); //SUBJECT TO CHANGE
                break;
            case LIFT:
                mArmEncoder.setPosition(0); //SUBJECT TO CHANGE(Unit is rotations)
                break;
            case HOLD:
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
                motorDutyCycle = -0.5;
                Logger.recordOutput("Climber/Velocity", mMotorVelocity);
                break;
            case HOLD:
                motorDutyCycle = -0.3; // A value that will hold place
                Logger.recordOutput("Climber/Position", mMotorPosition);
                break;
        }

        mArm.set(motorDutyCycle);
    }

    public enum ClimberState implements SubsystemState{
        IDLE, LIFT, HOLD;
    }
    
}
