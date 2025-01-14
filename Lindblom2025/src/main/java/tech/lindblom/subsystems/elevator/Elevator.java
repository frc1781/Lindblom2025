package tech.lindblom.subsystems.elevator;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;


public class Elevator extends StateSubsystem {
    private SparkMax motorRight, motorLeft;
    private RelativeEncoder rightEncoder = motorRight.getEncoder();
    private RelativeEncoder leftEncoder = motorLeft.getEncoder();

    public Elevator() {
        super("Elevator", ElevatorState.SAFE);
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig motorRightConfig = new SparkMaxConfig();
        SparkMaxConfig motorLeftConfig = new SparkMaxConfig();

        //Motor Config
        motorRightConfig.idleMode(IdleMode.kBrake);
        motorLeftConfig.idleMode(IdleMode.kBrake);
        motorRightConfig.smartCurrentLimit(40);
        motorLeftConfig.smartCurrentLimit(40);
        motorLeftConfig.follow(motorRight, true);
        motorRightConfig.softLimit.forwardSoftLimitEnabled(false);
        motorRightConfig.softLimit.forwardSoftLimitEnabled(false);


        //PID Config
        motorRightConfig.closedLoop.p(0);
        motorRightConfig.closedLoop.i(0);
        motorRightConfig.closedLoop.d(0);

        //Encoder Config
        motorRightConfig.encoder.positionConversionFactor(0);
        motorRightConfig.encoder.velocityConversionFactor(0);

        rightEncoder.setPosition(0.0);
        leftEncoder.setPosition(0.0);

        motorRight.configure(motorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motorLeft.configure(motorLeftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }


    @Override
    public boolean matchesState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'matchesState'");
    }


    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }


    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    public enum ElevatorState implements SubsystemState {
        L1,
        L2,
        L3,
        L4,
        SAFE
    }
}
