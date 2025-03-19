package tech.lindblom.subsystems.elevator;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.arm.Arm;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import java.util.HashMap;


public class Elevator extends StateSubsystem {
    private final SparkMax motorRight;

    private SparkMax motorLeft;

    private TimeOfFlight firstStageTOF;
    private TimeOfFlight secondStageTOF;
    // measure the max distance
    private double minSecondStageDistance = 0;
    private double maxSecondStageDistance = 680;

    private double minFirstStageDistance = 0;
    private double maxFirstStageDistance = 810; 

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward
            (Constants.Elevator.ELEVATOR_KS,
                    Constants.Elevator.ELEVATOR_KG,
                    Constants.Elevator.ELEVATOR_KV,
                    Constants.Elevator.ELEVATOR_KA
            );

    private final HashMap<ElevatorState, Double[]> positions = new HashMap<>();
    private RobotController robotController;

    public Elevator(RobotController robotController) {
        super("Elevator", ElevatorState.SAFE);
        this.robotController = robotController;
        firstStageTOF = new TimeOfFlight(Constants.Elevator.FIRST_STAGE_TOF);
        firstStageTOF.setRangingMode(TimeOfFlight.RangingMode.Short, 20);
        secondStageTOF = new TimeOfFlight(Constants.Elevator.SECOND_STAGE_TOF);
        secondStageTOF.setRangingMode(TimeOfFlight.RangingMode.Short, 20);

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


        positions.put(ElevatorState.POLE, new Double[]{750.0, minSecondStageDistance});
        positions.put(ElevatorState.SAFE, new Double[]{minFirstStageDistance, 80.0});
        positions.put(ElevatorState.SAFER, new Double[]{minFirstStageDistance, 80.0});
        positions.put(ElevatorState.L1, new Double[]{0.0, 0.0});
        positions.put(ElevatorState.L2, new Double[]{minFirstStageDistance, 80.0});
        positions.put(ElevatorState.L3, new Double[]{165.0, minSecondStageDistance});
        positions.put(ElevatorState.L4, new Double[]{maxFirstStageDistance, minSecondStageDistance});
        positions.put(ElevatorState.COLLECT_LOW, new Double[]{minFirstStageDistance, 400.0});
        positions.put(ElevatorState.GROUND_COLLECT, new Double[]{0.0, 290.0});
        positions.put(ElevatorState.HIGH_ALGAE, new Double[]{minFirstStageDistance, 50.0});
        positions.put(ElevatorState.LOW_ALGAE, new Double[]{maxFirstStageDistance, 200.0});
    }

    @Override
    public boolean matchesState() {
        if (getCurrentState() == ElevatorState.MANUAL_DOWN || getCurrentState() == ElevatorState.MANUAL_UP) {
            return false;
        }

        if (getCurrentState() == ElevatorState.POLE && robotController.getCenteringSide() != null) {
            return robotController.driveController.matchesState();
        }

        return matchesPosition();
    }

    public boolean matchesPosition() {
        if (RobotBase.isSimulation()) {
            return timeInState.get() > 3;
        }

        Double[] desiredPosition = positions.get(getCurrentState());
        double firstStageDiff = Math.abs(desiredPosition[0] - getFirstStagePosition());
        double secondStageDiff = Math.abs(desiredPosition[1] - getSecondStagePosition());
        double tolerance = 80;
        return firstStageDiff <= tolerance && secondStageDiff <= tolerance;
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/FirstStageTOF", firstStageTOF.getRange());
        Logger.recordOutput(this.name + "/SecondStageTOF", secondStageTOF.getRange());
        Logger.recordOutput(this.name + "/FirstStageTOFvalid", firstStageTOF.isRangeValid());
        Logger.recordOutput(this.name + "/SecondStageTOFvalid", secondStageTOF.isRangeValid());
        Logger.recordOutput(this.name + "/ElevatorMotorEncoderCounts", motorRight.getEncoder().getPosition());
        Logger.recordOutput(this.name + "/MatchesPosition", matchesPosition());

        if (currentOperatingMode == OperatingMode.DISABLED) return;

        if (robotController.isManualControlMode()) {
            switch ((ElevatorState) getCurrentState()) {
                case SAFE:
                    motorRight.set(0.02);
                    break;
                case MANUAL_DOWN:
                    motorRight.set(-0.5);
                    break;
                case MANUAL_UP:
                    motorRight.set(0.5);
                    break;
                default:
                    break;
            }
        } else if (positions.containsKey(getCurrentState())) {
            goToPosition();
        }
    }

    public double getFirstStagePosition() {
        return firstStageTOF.getRange();
    }

    public double getSecondStagePosition() {
        return secondStageTOF.getRange();
    }

    public void goToPosition() {
        double firstStagePosition = getFirstStagePosition();
        double secondStagePosition = getSecondStagePosition();
        double dutyCycle = 0;
        Double[] desiredPosition = positions.get(getCurrentState());
        double Tolerance = 80;

        
        if (secondStageTOF.isRangeValid() && Math.abs(desiredPosition[1] - secondStagePosition) >= Tolerance) {
            double ff = -feedforwardController.calculate(desiredPosition[1] - secondStagePosition);
            Logger.recordOutput(this.name + "/FFUnClamped", ff);
            double clampedResult = clampDutyCycle(ff);
            Logger.recordOutput(this.name + "/FFClampedOutput", clampedResult);
            dutyCycle = clampedResult;
        } else if (firstStageTOF.isRangeValid() && Math.abs(desiredPosition[0] - firstStagePosition) > Tolerance) {
            double ff = feedforwardController.calculate(desiredPosition[0] - firstStagePosition);
            Logger.recordOutput(this.name + "/FFUnClamped", ff);
            double clampedResult = clampDutyCycle(ff);
            Logger.recordOutput(this.name + "/FFClampedOutput", clampedResult);
            dutyCycle = clampedResult;
        } else {
            dutyCycle = 0.02;
        }

        //--------------------------------------------------------------
        //The second stage of the elevator (the inner stage) actually moves up first.
        //Well, the arm moves up until it hits the top of the 2nd stage then that stage starts going up.
        //If the arm is at the top of the second stage the first stage can move.  If it is not at the top of the
        //second stage then the second stage should not move unless it is out of the way of hitting the top.
        //It really only starts low at the beginning when it is not safe to move the second stage until the arm is moved
        //out.  But once it is up at the top of the second stage it can move into positions that make it dangerous
        //to leave it's spot on the second stage until it is back in a safe position.
        if ((!robotController.isSafeForElevatorStage2toMove() || !robotController.driveController.isSafeForElevatorStage2toMove()) && Math.abs(secondStagePosition - desiredPosition[1]) > 100) {
            dutyCycle = 0.02;
        }

        Logger.recordOutput(this.name + "/DutyCycle", dutyCycle);
        motorRight.set(dutyCycle);
    }

    public double clampDutyCycle(double dutyCycle) {
        if (getCurrentState() == ElevatorState.COLLECT_LOW || getCurrentState() == ElevatorState.L3 || getCurrentState() == ElevatorState.L2) {
            return EEUtil.clamp(-0.5, 0.5, dutyCycle);
        }

        return EEUtil.clamp(0, 0.5, dutyCycle);
    }

    public enum ElevatorState implements SubsystemState {
        SAFE,
        SAFER,
        L1,
        L2,
        L3,
        L4,
        MANUAL_DOWN,
        MANUAL_UP,
        COLLECT_LOW,
        POLE,
        GROUND_COLLECT,
        HIGH_ALGAE,
        LOW_ALGAE,
    }
}