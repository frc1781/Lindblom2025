package tech.lindblom.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;


public class Elevator extends StateSubsystem {
    private SparkMax motorRight;
    private RelativeEncoder rightEncoder;
    private SparkClosedLoopController rightPIDController;
    private SparkMax motorLeft;
    private RelativeEncoder leftEncoder;
    private SparkClosedLoopController leftPIDController;

    public Elevator() {
        super("Elevator", ElevatorState.SAFE);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        elevatorConfig.closedLoop.pid(0,0,0);
        elevatorConfig.closedLoop.iZone(0);
        elevatorConfig.closedLoop.minOutput(0);
        elevatorConfig.closedLoop.maxOutput(0);

        elevatorConfig.smartCurrentLimit(0);

        elevatorConfig.idleMode(IdleMode.kBrake);

        //Right Elevator Motor
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
        rightEncoder = motorRight.getEncoder();
        rightPIDController = motorRight.getClosedLoopController();
        motorRight.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Left Elevator Motor
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        leftEncoder = motorLeft.getEncoder();
        leftPIDController = motorLeft.getClosedLoopController();
        motorLeft.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum ElevatorState implements SubsystemState {
        SAFE,
        L1,
        L2,
        L3,
        L4
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
