package tech.lindblom.subsystems.elevator;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.SwerveModuleConfiguration;


public class Elevator extends StateSubsystem {
    private SparkMax motorRight, motorLeft;

    private AbsoluteEncoder elevatorAbsoluteEncoder;


    public Elevator() {
        super("Elevator", ArmState.SAFE);
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);

        //Motor Config
        SparkMaxConfig motorRightConfig = new SparkMaxConfig();
        motorRight.configure(motorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        SparkMaxConfig motorLeftConfig = new SparkMaxConfig();
        motorLeft.configure(motorLeftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        //PID Config
        motorRightConfig.closedLoop.p(0);
        motorRightConfig.closedLoop.i(0);
        motorRightConfig.closedLoop.d(0);
        motorLeftConfig.follow(motorRight);

        //Encoder Config
        motorRightConfig.encoder.positionConversionFactor(0);
        motorLeftConfig.encoder.positionConversionFactor(0);

        
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
}
