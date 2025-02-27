package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    private ArmState previousState;

    public Arm(RobotController controller) {
        super("Arm", ArmState.IDLE);

        coralTimeOfFlight = new TimeOfFlight(Constants.Arm.CLAW_CORAL_SENSOR_ID);
        robotController = controller;
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        armMotor.setControlFramePeriodMs(20);
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(30);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        armMotorConfig.absoluteEncoder.zeroOffset(.4457963);
        armMotorConfig.closedLoop.pid(0.01, 0,0.001);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-0.5, 0.5);
        armMotorConfig.closedLoop.positionWrappingEnabled(true);
        armMotorConfig.softLimit.forwardSoftLimit(180);
        armMotorConfig.softLimit.reverseSoftLimit(0);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positionMap = new HashMap<>();
        positionMap.put(ArmState.POLE, 5.0);
        positionMap.put(ArmState.IDLE, 10.0);
        positionMap.put(ArmState.L1, 45.0);
        positionMap.put(ArmState.L2, 0.0);
        positionMap.put(ArmState.L3, 90.0);
        positionMap.put(ArmState.L4, 80.0);
        positionMap.put(ArmState.WAIT, 25.0);
        positionMap.put(ArmState.COLLECT, 175.0);
    }

    @Override
    public boolean matchesState() {
        if (getCurrentState() == ArmState.MANUAL_DOWN || getCurrentState() == ArmState.MANUAL_UP) {
            return true;
        }

        if (getCurrentState() == ArmState.POLE) {
            return robotController.driveController.matchesState();
        }

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

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/MotorEncoder", armMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput(this.name + "/ArmTOF", coralTimeOfFlight.getRange());
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
            if (getCurrentState() == ArmState.IDLE && coralTimeOfFlight.getRange() < 20.0 && currentOperatingMode == OperatingMode.TELEOP) {
                setState(ArmState.COLLECT);
            }

            getToPosition(positionMap.get(getCurrentState()));
        }
    }

    @Override
    public void stateTransition(SubsystemState previousState, SubsystemState newState) {
        this.previousState = (ArmState) previousState;
    }

    private void getToPosition(double position){
        if ((getCurrentState() == defaultState && !robotController.isSafeForArmToMove()) || preventDescore()) return;
        armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
        Logger.recordOutput(this.name + "/Motor Duty Cycle", armMotor.get());
    }

    private boolean preventDescore() {
        return (getCurrentState() == defaultState && (previousState == ArmState.L3 || previousState == ArmState.L2 || previousState == ArmState.L1) && timeInState.get() < 2);
    }

    public enum ArmState implements SubsystemState {
        IDLE, L1, L2, L3, L4, MANUAL_UP, MANUAL_DOWN, COLLECT, WAIT, POLE
    }
}
