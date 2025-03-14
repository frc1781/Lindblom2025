package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    public Arm(RobotController controller) {
        super("Arm", ArmState.COLLECT);

        coralTimeOfFlight = new TimeOfFlight(Constants.Arm.CLAW_CORAL_SENSOR_ID);
        coralTimeOfFlight.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
        robotController = controller;
        timeCoralTOFInvalid = new Timer(); 
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        armMotor.setControlFramePeriodMs(20);
        //New config based on: https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/Configs.java
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(30);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        armMotorConfig.absoluteEncoder.zeroOffset(0.0);
        armMotorConfig.closedLoop.pid(0.01, 0,0.001);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-0.1, 0.1);
        armMotorConfig.closedLoop.positionWrappingEnabled(true);
        armMotorConfig.softLimit.forwardSoftLimit(180);
        armMotorConfig.softLimit.reverseSoftLimit(0);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        // armMotorConfig
        //     .idleMode(SparkMaxConfig.IdleMode.kBrake)
        //     .softLimit
        //         .forwardSoftLimit(180)
        //         .reverseSoftLimit(0);
        // armMotorConfig.smartCurrentLimit(30)
        //     .absoluteEncoder.positionConversionFactor(360)
        //         .zeroOffset(.4457963);
        // armMotorConfig
        //     .closedLoop
        //         .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        //         .pid(0.01, 0,0.001)
        //         .velocityFF((double) 1 /565) // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        //         .outputRange(-0.5, 0.5)  //modify depending on what is going on
        //         .positionWrappingEnabled(true)
        //         .maxMotion
        //             .maxVelocity(4200)
        //             .maxAcceleration(6000)
        //             .allowedClosedLoopError(0.5);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positionMap = new HashMap<>();
        // positionMap.put(ArmState.POLE, 17.0);
        // positionMap.put(ArmState.IDLE, 25.0);
        // positionMap.put(ArmState.L1, 45.0);
        // positionMap.put(ArmState.L2, 0.0);
        // positionMap.put(ArmState.L3, 70.0);
        // positionMap.put(ArmState.L4, 80.0);
        // positionMap.put(ArmState.WAIT, 25.0);
        // positionMap.put(ArmState.COLLECT, 175.0);
        positionMap.put(ArmState.POLE, 40.0);
        positionMap.put(ArmState.IDLE, 40.0);
        positionMap.put(ArmState.L1, 40.0);
        positionMap.put(ArmState.L2, 40.0);
        positionMap.put(ArmState.L3, 40.0);
        positionMap.put(ArmState.L4, 40.0);
        positionMap.put(ArmState.WAIT, 40.0);
        positionMap.put(ArmState.COLLECT, 40.0);
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
            double tolerance = 6;
            Logger.recordOutput(this.name + "/DesiredPositionDifference", Math.abs(positionMap.get(getCurrentState()) - armMotor.getAbsoluteEncoder().getPosition()));
            return Math.abs(positionMap.get(getCurrentState()) - armMotor.getAbsoluteEncoder().getPosition()) <= tolerance;
        }
        return false;
    }


    @Override
    public void init() {
       
    }

    public double getPosition() {
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/MotorEncoder", armMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput(this.name + "/coralTOF", coralTimeOfFlight.getRange());
        Logger.recordOutput(this.name + "/coralTOFisValid", coralTimeOfFlight.isRangeValid());
        Logger.recordOutput(this.name + "/hasCoral", hasCoral());
        Logger.recordOutput(this.name + "/isSafeForElevatorToMove", isSafeForElevatorToMove());

        if(currentOperatingMode == OperatingMode.DISABLED) return;
        if (robotController.isManualControlMode()) {
            switch ((ArmState) getCurrentState()) {
                case IDLE:
                    armMotor.set(0);
                    break;
                case MANUAL_DOWN:
                    armMotor.set(-0.1);
                    break;
                case MANUAL_UP:
                    armMotor.set(0.1);
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

    @Override
    public ArmState getDefaultState() {
        if (robotController.isManualControlMode()) {
            return ArmState.IDLE;
        }

        if (hasCoral()) {
            return ArmState.POLE;
        } else {
            return ArmState.COLLECT;
        }
    }

    public boolean isSafeForElevatorToMove() {
        return getPosition() > 40.0 && getPosition() < 300;  //should never be this high except with gimble lock wrapping 
    }

    private void getToPosition(double position){
        if ((getCurrentState() == getDefaultState() && !robotController.isSafeForArmToMove()) || preventDescore()) {
            armMotor.set(0);
            return;
        }
        armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
        Logger.recordOutput(this.name + "/Motor Duty Cycle", armMotor.get());
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

        return coralTimeOfFlight.getRange() > 10 && coralTimeOfFlight.getRange() < 100;
    }


    private boolean preventDescore() {
        return (getCurrentState() == getDefaultState() && (previousState == ArmState.L4 || previousState == ArmState.L3 || previousState == ArmState.L2 || previousState == ArmState.L1)
                && timeInState.get() < 2
        );
    }

    public enum ArmState implements SubsystemState {
        IDLE, L1, L2, L3, L4, MANUAL_UP, MANUAL_DOWN, COLLECT, WAIT, POLE
    }
}
