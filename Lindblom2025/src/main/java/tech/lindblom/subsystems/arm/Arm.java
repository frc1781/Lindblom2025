package tech.lindblom.subsystems.arm;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.EnumCollection;
import tech.lindblom.utils.EnumCollection.OperatingMode;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import tech.lindblom.control.RobotController;
import tech.lindblom.utils.Constants;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAlternateEncoder;

public class Arm extends StateSubsystem {
    private CANSparkMax mRightMotor, mLeftMotor;

    private AbsoluteEncoder mArmAbsoluteEncoder;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.03, 0, 0,
            new TrapezoidProfile.Constraints(90, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();

    private ProfiledPIDController mSmallPosHold = new ProfiledPIDController(0.01, 0, 0,
            new TrapezoidProfile.Constraints(90, 450));
    //kG
    //kS
    //kV

    private double mDesiredPosition = 0;
    private Pose2d mRobotPose;
    private double KICKSTAND_POSITION = 70.0; // was 73 Was 62.0
    private double mPrevAbsoluteAngle = KICKSTAND_POSITION;
    private double mPrevRecordedAngle = 0.0;
    private IdleMode mIdleMode;

    private double armDutyCycle;
    public Arm() {
        super("Arm", ArmState.SAFE);
        mRightMotor = new CANSparkMax(
                Constants.Arm.ARM_PIVOT_RIGHT_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        mRightMotor.restoreFactoryDefaults();
        mLeftMotor = new CANSparkMax(
                Constants.Arm.ARM_PIVOT_LEFT_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setSmartCurrentLimit(40);
        mRightMotor.setSmartCurrentLimit(40);
        mArmAbsoluteEncoder = mRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        //mArmAbsoluteEncoder.setAverageDepth(1);
        mArmAbsoluteEncoder.setInverted(true);
        mRightMotor.follow(mLeftMotor, true);

        mIdleMode = IdleMode.kBrake;
        mRightMotor.setIdleMode(mIdleMode);
        mLeftMotor.setIdleMode(mIdleMode);
      
        mRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); //important for absoulte encoder to respond quicky

        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
       // mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 80);
       // mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -10);
        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();

        mPositions.put(ArmState.SAFE, 70.0); 
        mPositions.put(ArmState.PODIUM, 50.0);
        mPositions.put(ArmState.SUBWOOFER, 35.0); // was 36
        mPositions.put(ArmState.AMP, 50.0); // Was 46.0
        mPositions.put(ArmState.COLLECT, 0.0);
        mPositions.put(ArmState.COLLECT_HIGH, 60.0); // Was 55.7
        mPositions.put(ArmState.SKIP, 55.0);
        mPositions.put(ArmState.KICKSTAND, 80.0);
        mPositions.put(ArmState.LOB, 35.0);
        mPositions.put(ArmState.NOTE_ONE, 48.0);
        mPositions.put(ArmState.NOTE_TWO, 48.0);
        mPositions.put(ArmState.NOTE_THREE, 48.0);
        mPositions.put(ArmState.FAR_SHOT, 58.0);
    }

    public enum ArmState implements SubsystemState {
        KICKSTAND,
        SAFE,
        PODIUM,
        SUBWOOFER,
        COLLECT,
        COLLECT_HIGH,
        MANUAL,
        AUTO_ANGLE,
        AMP,
        SKIP,
        NOTE_ONE,
        NOTE_TWO,
        NOTE_THREE,
        LOB,
        FAR_SHOT
    }

    @Override
    public boolean matchesState() {
        switch ((ArmState) getCurrentState()) {
            case COLLECT:
                return getAngle() < 4.0; // should fall to position of zero
            case KICKSTAND:
                return mLeftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed() || matchesPosition();
            default:
                return matchesPosition();
        }
    }

    @Override
    public void init() {
        
    }

    @Override
    public void periodic() {
        
    }

    private boolean matchesPosition() {
        return Math.abs(getAngleAbsolute() - mDesiredPosition) < 1.85;
    }

    private double getAngle() {
        return getAngleAbsolute();
    }

    private double getAngleAbsolute() {
        double reportedPosition = mArmAbsoluteEncoder.getPosition();
        if (reportedPosition > 0.1) {
            mPrevAbsoluteAngle = 360.0 * (mArmAbsoluteEncoder.getPosition() - Constants.Arm.ARM_OFFSET); // the absolute encoder reads
        }
        return mPrevAbsoluteAngle;
    }


}
