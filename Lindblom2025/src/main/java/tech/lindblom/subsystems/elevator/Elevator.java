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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import java.util.HashMap;


public class Elevator extends StateSubsystem {
    private final SparkMax motorRight;

    private SparkMax motorLeft;

    private TimeOfFlight firstStageTOF;
    private TimeOfFlight secondStageTOF;
    private TimeOfFlight lowerTroughTOF;
    // measure the max distance
    private LoggedMechanism2d elevatorMechSimulation;

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
        lowerTroughTOF = new TimeOfFlight(Constants.Elevator.LOWER_TROUGH__TOF);

        //Right Elevator Motor
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(30);
        motorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Left Elevator Motor
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.follow(motorRight, true);
        leftMotorConfig.smartCurrentLimit(30);
        motorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(ElevatorState.SAFE, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L1, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L2, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L3, new Double[]{0.0,0.0});
        positions.put(ElevatorState.L4, new Double[]{0.0,0.0});

        elevatorMechSimulation = new LoggedMechanism2d(3,3);
    }


    @Override
    public boolean matchesState() {
        Double[] desiredPosition = positions.get(getCurrentState());
        double firstStageDiff = Math.abs(desiredPosition[0] - getFirstStagePosition());
        double secondStageDiff = Math.abs(desiredPosition[0] - getSecondStagePosition());
        double tolerance = 25;
        return firstStageDiff <= tolerance && secondStageDiff <= tolerance;
    }


    @Override
    public void init() {
    }


    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/SimulationMech", elevatorMechSimulation);
        Logger.recordOutput(this.name + "/FirstStageTOF", firstStageTOF.getRange());
        Logger.recordOutput(this.name + "/SecondStageTOF", secondStageTOF.getRange());
        Logger.recordOutput(this.name + "/LowerTroughTOF", lowerTroughTOF.getRange());
        Logger.recordOutput(this.name + "/ElevatorMotorEncoderCounts", motorRight.getEncoder().getPosition());

        if (currentMode == OperatingMode.DISABLED) return;
        switch ((ElevatorState) getCurrentState()) {
            case SAFE:
                motorRight.set(0);
                break;

            case MANUAL_DOWN:
                motorRight.set(-0.1);
                break;
            case MANUAL_UP:
                motorRight.set(0.1);
                break;
        }
        //goToPosition();
    }

    public double getFirstStagePosition() {
        return firstStageTOF.getRange();
    }

    public double getSecondStagePosition() {
        return secondStageTOF.getRange();
    }

    public boolean hasCoral() {
        return lowerTroughTOF.getRange() <= 125;
    }

    public void goToPosition() {
        double totalPosition = getFirstStagePosition() + getSecondStagePosition();
        double desiredPosition = positions.get(getCurrentState())[0] + positions.get(getCurrentState())[1];
        double ff = feedforwardController.calculate(desiredPosition - totalPosition);
        if (ff <= 0) {
            ff = 0;
        }

        if (ff > 0.1) {
            ff = 0.1;
        }

        motorRight.set(ff);
    }

    public enum ElevatorState implements SubsystemState {
        SAFE,
        L1,
        L2,
        L3,
        L4,
        MANUAL_DOWN,
        MANUAL_UP,
    }
}
