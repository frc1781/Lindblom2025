package tech.lindblom.subsystems.elevator;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import java.util.HashMap;


public class Elevator extends StateSubsystem {
    private final SparkMax motorRight;

    private SparkMax motorLeft;

    private TimeOfFlight firstStageTOF;
    private TimeOfFlight secondStageTOF;
    // measure the max distance

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward
            (Constants.Elevator.ELEVATOR_KS,
                    Constants.Elevator.ELEVATOR_KG,
                    Constants.Elevator.ELEVATOR_KV,
                    Constants.Elevator.ELEVATOR_KA
            );

    private final HashMap<ElevatorState, Double[]> positions = new HashMap<>();

    public Elevator() {
        super("Elevator", ElevatorState.SAFE);

        firstStageTOF = new TimeOfFlight(Constants.Elevator.FIRST_STAGE_TOF);
        secondStageTOF = new TimeOfFlight(Constants.Elevator.SECOND_STAGE_TOF);


        //Right Elevator Motor
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);

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

        positions.put(ElevatorState.SAFE, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L1, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L2, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L3, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L4, new Double[]{0.0,0.0});
    }


    @Override
    public boolean matchesState() {
        Double[] desiredPosition = positions.get(getCurrentState());
        double firstStageDiff = Math.abs(desiredPosition[0] - getFirstStagePosition());
        double secondStageDiff = Math.abs(desiredPosition[0] - getSecondStagePosition());
        double tolerance = 25;
        return firstStageDiff <= 25 && secondStageDiff <= 25;
    }


    @Override
    public void init() {
    }


    @Override
    public void periodic() {
        if (currentMode == OperatingMode.DISABLED) return;
        goToPosition();
    }

    public double getFirstStagePosition() {
        return firstStageTOF.getRange();
    }

    public double getSecondStagePosition() {
        return secondStageTOF.getRange();
    }

    public void goToPosition() {
        double totalPosition = getFirstStagePosition() + getSecondStagePosition();
        double desiredPosition = positions.get(getCurrentState())[0] + positions.get(getCurrentState())[1];
        double ff = feedforwardController.calculate(desiredPosition - totalPosition);
        motorRight.set(ff);
    }

    public enum ElevatorState implements SubsystemState {
        SAFE,
        L1,
        L2,
        L3,
        L4
    }
}
