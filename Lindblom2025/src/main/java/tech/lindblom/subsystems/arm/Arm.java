package tech.lindblom.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.HashMap;

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

    public Arm(RobotController controller) {
        super("Arm", ArmState.IDLE);

        robotController = controller;
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(30);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360.0);
        armMotorConfig.closedLoop.pid(0.01, 0,0.001);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-0.5, 0.5);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positionMap = new HashMap<>();
        positionMap.put(ArmState.IDLE, 25.0);
        positionMap.put(ArmState.L1, 45.0);
        positionMap.put(ArmState.L2, 0.0);
        positionMap.put(ArmState.L3, 0.0);
        positionMap.put(ArmState.L4, 90.0);
        positionMap.put(ArmState.WAIT, 25.0);
        positionMap.put(ArmState.COLLECT, 195.0);
    }

    @Override
    public boolean matchesState() {
       double tolerance = 6;
       Logger.recordOutput(this.name + "/DesiredPositionDifference", Math.abs(positionMap.get(getCurrentState()) - armMotor.getAbsoluteEncoder().getPosition()));
       return Math.abs(positionMap.get(getCurrentState()) - armMotor.getAbsoluteEncoder().getPosition()) <= tolerance;
    }

    @Override
    public void init() {
       
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/MotorEncoder", armMotor.getAbsoluteEncoder().getPosition());
        if(currentMode == OperatingMode.DISABLED) return;
        switch ((ArmState) getCurrentState()) {
            case MANUAL_DOWN:
                armMotor.set(-0.1);
                break;
            case MANUAL_UP:
                armMotor.set(0.1);
                break;
             default:
                getToPosition(positionMap.get(getCurrentState()));
                break;

        }
    }

    private void getToPosition(double position ){
        if (getCurrentState() == defaultState && !robotController.isSafeForArmToMove()) return;
        armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
        Logger.recordOutput(this.name + "/Motor Duty Cycle", armMotor.get());
    }

    public enum ArmState implements SubsystemState {
        IDLE, L1, L2, L3, L4, MANUAL_UP, MANUAL_DOWN, COLLECT, WAIT
    }
}
