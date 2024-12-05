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
import edu.wpi.first.math.geometry.Rotation2d;
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
    private CURRENT_AIM_SPOT mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
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

    public void getToState() {
        switch ((ArmState) getCurrentState()) {
            case AUTO_ANGLE:
                mDesiredPosition = calculateAngleFromDistance();
                break;
            case MANUAL:
                break;
            default:
                mDesiredPosition = mPositions.get(getCurrentState());
                break;
        }

        double currentArmAngle = getAngle();
        //mArmPositionEntry.setDouble(currentArmAngle);
        if (currentArmAngle != 0.0) {
            var armDutyCycle = mPositionPID.calculate(currentArmAngle, mDesiredPosition);
            // if (mSparkErrorEntry.getBoolean(false))
            //     mSparkErrorEntry.setBoolean(false);

            if (getCurrentState() == ArmState.COLLECT && getAngle() < 10.0) { // drop into position on ground
                armDutyCycle = 0.0;
            }

            mLeftMotor.set(armDutyCycle);
        } else {
            //mSparkErrorEntry.setBoolean(true);
        }
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
        setDesiredState(ArmState.KICKSTAND);
        syncArm();
        mPositionPID.reset(getAngle());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/MatchesState", matchesState());
        Logger.recordOutput("Arm/RawAbsoluteArm", getAngleAbsolute());

        // testEntry.setDouble(getAngleAbsolute());
        if (mArmAbsoluteEncoder.getPosition() < 10) {
            setIdleMode(IdleMode.kCoast);
        } else {
            setIdleMode(IdleMode.kBrake);
        }


        Logger.recordOutput("Arm/CurrentAimSpot",mCurrentAimSpot.toString());

        //dropped to ground, reset relative encoder only when going down.
        syncArm();
    }

    public void setDesiredState(SubsystemState state) {
        if (state == ArmState.SAFE && state == ArmState.MANUAL) {
            return;
        }

        super.setState(state);

        if (state != ArmState.MANUAL && state != ArmState.AUTO_ANGLE) {
            mDesiredPosition = mPositions.get(state);
        } else if (state == ArmState.AUTO_ANGLE) {
            mDesiredPosition = calculateAngleFromDistance();
        }
    }

    private void setIdleMode(IdleMode mode) {
        if (mode == mIdleMode) {
            return;
        }

        mLeftMotor.setIdleMode(mode);
        mRightMotor.setIdleMode(mode);
        System.out.println(mode);
        mIdleMode = mode;
    }

    private void syncArm() {
        double abs = getAngleAbsolute();
        if(abs != mPrevRecordedAngle) {
            mPrevRecordedAngle = abs;
        }

    }

    public void updateAimSpots(Pose2d robotPose) {
        mRobotPose = robotPose;
    }

    private double calculateAngleFromDistance() {
        boolean foundAimSpot = false;

        for (CURRENT_AIM_SPOT aimSpot : CURRENT_AIM_SPOT.values()) {
            if (aimSpot == CURRENT_AIM_SPOT.UNDEFEINED)
                continue;
            if (aimSpot.atPosition(mRobotPose)) {
                mCurrentAimSpot = aimSpot;
                foundAimSpot = true;
                break;
            }
        }

        if (!foundAimSpot) {
            mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
        }

        if (mCurrentAimSpot != CURRENT_AIM_SPOT.UNDEFEINED) {
            return mCurrentAimSpot.getPosition();
        }

        return CURRENT_AIM_SPOT.SUBWOOFER.getPosition();
    }


    public void manualAdjustAngle(double d) {
        setDesiredState(ArmState.MANUAL);

        mDesiredPosition += d;
        if (mDesiredPosition > Constants.Arm.MAX_THRESHOLD_ARM) {
            mDesiredPosition = Constants.Arm.MAX_THRESHOLD_ARM;
        }

        if (mDesiredPosition < Constants.Arm.MIN_THRESHOLD_ARM) {
            mDesiredPosition = Constants.Arm.MIN_THRESHOLD_ARM;
        }
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

    private enum CURRENT_AIM_SPOT {
        UNDEFEINED(0.0, new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), 0.0),
        SUBWOOFER(35, Constants.RED_SPEAKER_LOCATION, Constants.BLUE_SPEAKER_LOCATION, 2.5), // Was 32.5
        PODIUM(50.0, Constants.RED_PODIUM, Constants.BLUE_PODIUM, 1), // Pos used to be 45
        NOTE_3(48, new Pose2d(14.5, 4.27, new Rotation2d(0)), new Pose2d(2.48, 4.27, new Rotation2d(0)), 1), // was 42.4
        NOTE_2(48, new Pose2d(14.13, 5.53, new Rotation2d(0)), new Pose2d(2.48, 5.53, new Rotation2d(0)), 0.5), // Was 50
        NOTE_1(48, new Pose2d(14.06, 6.74, new Rotation2d(0)), new Pose2d(2.48, 6.74, new Rotation2d(0)), 0.5); // Was 50

        private double position;
        private Pose2d redPosition;
        private Pose2d bluePosition;
        private double distanceTolerance;

        private CURRENT_AIM_SPOT(double _position, Pose2d _redPosition, Pose2d _bluePosition,
                double _distanceTolerance) {
            position = _position;
            redPosition = _redPosition;
            bluePosition = _bluePosition;
            distanceTolerance = _distanceTolerance;
        }

        public boolean atPosition(Pose2d currentPose2d) { //Tells whether it is at the subwoofer for red or blue or not
            Pose2d target = RobotController.isRed() ? redPosition : bluePosition;
            double dist = Math.sqrt(Math.pow(target.getX() - currentPose2d.getX(), 2) + Math.pow(target.getY() - currentPose2d.getY(), 2));
            return dist <= distanceTolerance;
        }

        public double getPosition() {
            return position;
        }
    }


}
