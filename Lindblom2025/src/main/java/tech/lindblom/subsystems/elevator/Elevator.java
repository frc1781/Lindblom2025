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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

import java.util.HashMap;


public class Elevator extends StateSubsystem {
    private final SparkMax motorRight;
    private RelativeEncoder rightEncoder;

    private SparkMax motorLeft;

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward
            (Constants.Elevator.ELEVATOR_KS,
                    Constants.Elevator.ELEVATOR_KG,
                    Constants.Elevator.ELEVATOR_KV,
                    Constants.Elevator.ELEVATOR_KA
            );

    private final HashMap<ElevatorState, Double> positions = new HashMap<>();

    public Elevator() {
        super("Elevator", ElevatorState.SAFE);

        //Right Elevator Motor
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
        rightEncoder = motorRight.getEncoder();

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.smartCurrentLimit(30);
        motorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Left Elevator Motor
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.inverted(true);
        leftMotorConfig.follow(motorRight);
        leftMotorConfig.smartCurrentLimit(30);
        motorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(ElevatorState.SAFE, 0.0);
        positions.put(ElevatorState.L1, 0.0);
        positions.put(ElevatorState.L2, 0.0);
        positions.put(ElevatorState.L3, 0.0);
        positions.put(ElevatorState.L4, 0.0);
    }


    @Override
    public boolean matchesState() {
        return getPosition() == positions.get(getCurrentState());
    }


    @Override
    public void init() {

    }


    @Override
    public void periodic() {
        switch ((ElevatorState) getCurrentState()) {
            case SAFE: 
            goToPosition(positions.get(ElevatorState.SAFE));
                break;
            case L1:
            goToPosition(positions.get(ElevatorState.L1));
                break;
            case L2:
            goToPosition(positions.get(ElevatorState.L2));
                break;
            case L3:
            goToPosition(positions.get(ElevatorState.L3));
                break;
            case L4:
            goToPosition(positions.get(ElevatorState.L4));
                break;
        }
    }

    public double getPosition() {
        return rightEncoder.getPosition();
    }

    public void goToPosition(double position) {
        double ff = feedforwardController.calculate((getPosition() - position));
        motorRight.setVoltage(ff);
    }

    public enum ElevatorState implements SubsystemState {
        SAFE,
        L1,
        L2,
        L3,
        L4
    }
}
