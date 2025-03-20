package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkLowLevel;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection.OperatingMode;

public class Arm extends StateSubsystem {
    private SparkMax armMotor;
    private HashMap<ArmState,Double> positionMap;
    private RobotController robotController;
    private TimeOfFlight coralTimeOfFlight;
    private Timer timeCoralTOFInvalid;
    private ArmState previousState;

    private boolean performedSafeStates = true;

    public Arm(RobotController controller) {
        super("Arm", ArmState.IDLE);

        coralTimeOfFlight = new TimeOfFlight(Constants.Arm.CLAW_CORAL_SENSOR_ID);
        coralTimeOfFlight.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        robotController = controller;
        timeCoralTOFInvalid = new Timer(); 
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        armMotor.setControlFramePeriodMs(20);
        //New config based on: https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/Configs.java
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(30);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        armMotorConfig.absoluteEncoder.zeroOffset(0.3209622);

        // Slot 0 configs
        armMotorConfig.closedLoop.pid(0.008, 0,0.001);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-.55, .55);
        armMotorConfig.closedLoop.positionWrappingEnabled(true);

        // Slot 1 configs
        armMotorConfig.closedLoop.pid(0.005, 0, 0.001, ClosedLoopSlot.kSlot1);
        armMotorConfig.closedLoop.outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot1);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565, ClosedLoopSlot.kSlot1);

        // Slot 2 configs | not used rn but here for funiess
        armMotorConfig.closedLoop.pid(0.095, 0, 0.001, ClosedLoopSlot.kSlot2);
        armMotorConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot2);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565, ClosedLoopSlot.kSlot2);

        armMotorConfig.softLimit.forwardSoftLimit(180);
        armMotorConfig.softLimit.reverseSoftLimit(0);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positionMap = new HashMap<>();
        positionMap.put(ArmState.POLE, 25.0);
        positionMap.put(ArmState.IDLE, 2.0);
        positionMap.put(ArmState.L1, 45.0);
        positionMap.put(ArmState.L2, 0.0);
        positionMap.put(ArmState.L3, 70.0);
        positionMap.put(ArmState.L4, 45.0);
        positionMap.put(ArmState.WAIT, 25.0);
        positionMap.put(ArmState.COLLECT, 185.0);
        positionMap.put(ArmState.START_HIGH, 5.0);
        positionMap.put(ArmState.START_MID, 40.0);
        positionMap.put(ArmState.GROUND_ALGAE, 159.0);
        positionMap.put(ArmState.REEF_ALGAE, 50.0);
        positionMap.put(ArmState.SLIGHT_TOSS, 21.0);
    }

    @Override
    public boolean matchesState() {
        if (getCurrentState() == ArmState.MANUAL_DOWN || getCurrentState() == ArmState.MANUAL_UP) {
            return true;
        }
        return matchesDesiredPosition();
    }

    public boolean matchesDesiredPosition() {
        if (positionMap.containsKey(getCurrentState())) {
            if (RobotBase.isSimulation()) {
                return timeInState.get() > 3;
            }

            double tolerance = 6;
            Logger.recordOutput(this.name + "/DesiredPositionDifference", Math.abs(positionMap.get(getCurrentState()) - armMotor.getAbsoluteEncoder().getPosition()));
            return Math.abs(positionMap.get(getCurrentState()) - armMotor.getAbsoluteEncoder().getPosition()) <= tolerance;
        }
        return false;
    }

    @Override
    public void init() {
        super.init();
       if (currentOperatingMode == OperatingMode.TELEOP && getPosition() < 30 && robotController.elevatorSystem.getSecondStagePosition() > 200) {
            performedSafeStates = false;
       }
    }

    public double getPosition() {
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/MatchesPosition", matchesDesiredPosition());
        Logger.recordOutput(this.name + "/MotorEncoder", armMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput(this.name + "/coralTOF", coralTimeOfFlight.getRange());
        Logger.recordOutput(this.name + "/coralTOFisValid", coralTimeOfFlight.isRangeValid());
        Logger.recordOutput(this.name + "/hasCoral", hasCoral());

        if(currentOperatingMode == OperatingMode.DISABLED) {
            return;
        }
 
        if (robotController.isManualControlMode()) {
            switch ((ArmState) getCurrentState()) {
                case IDLE:
                    armMotor.set(0);
                    break;
                case MANUAL_DOWN:
                    armMotor.set(-0.3);
                    break;
                case MANUAL_UP:
                    armMotor.set(0.3);
                    break;
            }
        } else if (positionMap.containsKey(getCurrentState())) {
                getToPosition(positionMap.get(getCurrentState()));
        }
    }

    @Override
    public void stateTransition(SubsystemState previousState, SubsystemState newState) {
        this.previousState = (ArmState) previousState;
    }

    public boolean successfullyCollectedAlgae() {
        return previousState == ArmState.REEF_ALGAE && hasCoral();
    }

    @Override
    public ArmState getDefaultState() {
        if (robotController.isManualControlMode() || currentOperatingMode == OperatingMode.DISABLED || currentOperatingMode == null) {
            return ArmState.IDLE;
        }

        if (!performedSafeStates) {
            // the perfect state machine, pls no touch - ally
            if (matchesDesiredPosition() && getCurrentState() == ArmState.START_HIGH) {
                performedSafeStates = true;
            }

            if (getCurrentState() == ArmState.START_HIGH) {
                return ArmState.START_HIGH;
            }

            if (matchesDesiredPosition() && getCurrentState() == ArmState.START_MID) {
                return ArmState.START_HIGH;
            }

            if (getCurrentState() == ArmState.IDLE || getCurrentState() == ArmState.START_MID) {
                return ArmState.START_MID;
            }
        } else {
            if (currentOperatingMode != OperatingMode.AUTONOMOUS) {
                if (hasCoral()) {
                    return ArmState.IDLE;
                } else {
                    return ArmState.COLLECT;
                }
            }
        }

        return ArmState.IDLE;
    }

    private boolean finishedStartingActions() {
        return performedSafeStates;
    }

    private void getToPosition(double position){
        if (getCurrentState() != ArmState.IDLE && !robotController.isSafeForArmToLeaveIdle() && getCurrentState() != ArmState.START_MID) {
            armMotor.set(0);
            return;
        }

        if (getCurrentState() == ArmState.COLLECT) {
            armMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } if (getCurrentState() == ArmState.POLE) {
            armMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot2);
        } else {
            armMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        Logger.recordOutput(this.name + "/dutyCycle", armMotor.getAppliedOutput());
    }

    public boolean hasCoral() {
        //TOF reported inValid for very short times even when valid, filtering out so only reporting invalid if
        //has been reporting invalid for more than 0.1 seconds.
        if (coralTimeOfFlight.isRangeValid()) {
            timeCoralTOFInvalid.reset();
        }
        else {
            if (!timeCoralTOFInvalid.isRunning()) {
                timeCoralTOFInvalid.start();
            }
        }

        if (timeCoralTOFInvalid.get() > 0.1) {
            return false;
        }

        return coralTimeOfFlight.getRange() < 50;
    }

    private boolean preventDescore() {
        return (getCurrentState() == getDefaultState() && (previousState == ArmState.L4 || previousState == ArmState.L3 || previousState == ArmState.L2 || previousState == ArmState.L1)
                && timeInState.get() < 2
        );
    }

    public enum ArmState implements SubsystemState {
        IDLE, L1, L2, L3, L4, MANUAL_UP, MANUAL_DOWN, COLLECT, WAIT, POLE, START_MID, START_HIGH, GROUND_ALGAE, REEF_ALGAE, SLIGHT_TOSS
    }
}
