package tech.lindblom.subsystems.thumb;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.arm.Arm.ArmState;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

import static tech.lindblom.control.RobotController.Action.THUMB_SPIN_IN;

import java.lang.annotation.Native;

public class Thumb extends StateSubsystem {
    private final SparkFlex spinMotor;
    private final RobotController robotController;
    private ThumbState prevState = ThumbState.IDLE;

    public Thumb(RobotController robotController) {
        super("Thumb", ThumbState.IDLE);

        this.robotController = robotController;
        //Spin Motor
        spinMotor = new SparkFlex(Constants.Thumb.SPIN_THUMB_MOTOR, MotorType.kBrushless);
        SparkFlexConfig spinMotorConfig = new SparkFlexConfig();
        spinMotorConfig.idleMode(IdleMode.kBrake);
        spinMotorConfig.smartCurrentLimit(30);
        spinMotorConfig.inverted(false);
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        if (prevState == ThumbState.SPIN_IN && getCurrentState() == ThumbState.IDLE && timeInState.get() < 1) {
            spinMotor.set(0.5);
            return;
        }

        switch ((ThumbState) getCurrentState()) {
            case SPIN_IN:
                spinMotor.set(-1);
                break;
            case SPIN_OUT:
                spinMotor.set(1);
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
    public ThumbState getDefaultState() {
        if ((ThumbState.IDLE == getCurrentState() || ThumbState.SPIN_IN == getCurrentState())&& robotController.armSystem.successfullyCollectedAlgae()) {
            return ThumbState.SPIN_IN;
        }

        return ThumbState.IDLE;
    }

    @Override
    public void stateTransition(SubsystemState previousState, SubsystemState newState) {
        this.prevState = (ThumbState) previousState;
    }

    @Override
    public void init() {
        super.init();
        spinMotor.set(0);
    }

    public enum ThumbState implements SubsystemState {
        SPIN_IN, SPIN_OUT, IDLE
    }
}