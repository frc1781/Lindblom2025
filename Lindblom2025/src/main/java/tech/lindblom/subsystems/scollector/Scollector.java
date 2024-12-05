package tech.lindblom.subsystems.scollector;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
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
import tech.lindblom.utils.Constants;
import tech.lindblom.control.RobotController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.utils.EnumCollection;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

public class Scollector extends StateSubsystem{

    private CANSparkMax mCollectorMotor = new CANSparkMax(Constants.Scollector.COLLECTOR_MOTOR, CANSparkMax.MotorType.kBrushless);
    private CANSparkFlex mTopShooterMotor = new CANSparkFlex(Constants.Scollector.SHOOTER_TOP_MOTOR,
            CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex mBottomShooterMotor = new CANSparkFlex(Constants.Scollector.SHOOTER_BOTTOM_MOTOR,
            CANSparkFlex.MotorType.kBrushless);

    private final SparkPIDController mTopPID;
    private final SparkPIDController mBottomPID;

    private final double SHOOTER_PID[] = {0.3, 0.0, 0.0};
    private final TrapezoidProfile.Constraints SHOOTER_CONSTRAINTS = new TrapezoidProfile.Constraints(9.0, 10);
    private ProfiledPIDController mBottomShooterPID = new ProfiledPIDController(SHOOTER_PID[0], SHOOTER_PID[1],
            SHOOTER_PID[2], SHOOTER_CONSTRAINTS);
    private ProfiledPIDController mTopShooterPID = new ProfiledPIDController(SHOOTER_PID[0], SHOOTER_PID[1],
            SHOOTER_PID[2], SHOOTER_CONSTRAINTS);

    private TimeOfFlight mTopTof = new TimeOfFlight(Constants.Scollector.TOP_SCOLLECTOR_TOF);
    private TimeOfFlight mBottomTof = new TimeOfFlight(Constants.Scollector.BOTTOM_SCOLLECTOR_TOF);

    private boolean mArmInPosition = false;
    private Timer mShooterTimer = new Timer();

    //TO BE CHANGED TO LOGGING VVVVVVVV

    // private GenericEntry mTopShooterVelocity = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Top Velocity", 0,
    //         ShuffleboardStyle.TOP_SHOOTER);
    // private GenericEntry mBottomShooterVelocity = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB,
    //         "Bottom Velocity", 0, ShuffleboardStyle.BOTTOM_SHOOTER);
    // private GenericEntry mReadyToShootEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Ready To Shoot",
    //         true, ShuffleboardStyle.READY_TO_SHOOT);
    // private GenericEntry mHasNoteEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Has Note", false,
    //         ShuffleboardStyle.HAS_NOTE);


    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
        mCollectorMotor.setIdleMode(IdleMode.kBrake);
        mBottomShooterMotor.setIdleMode(IdleMode.kCoast);
        mTopShooterMotor.setIdleMode(IdleMode.kCoast);
        mTopShooterMotor.setInverted(false);
        mBottomShooterMotor.setInverted(true);

        final double conversionFactor = 0.100203 * 1 / 60;

        mBottomShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mBottomShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        mTopShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mTopShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        mBottomShooterMotor.setSmartCurrentLimit(30);
        mTopShooterMotor.setSmartCurrentLimit(30);
        mTopPID = mTopShooterMotor.getPIDController();
        mBottomPID = mBottomShooterMotor.getPIDController();
        mTopPID.setFeedbackDevice(mTopShooterMotor.getEncoder());
        mBottomPID.setFeedbackDevice(mBottomShooterMotor.getEncoder());

        final double[] SHOOTER_PID = {0.5, 0, 0.1};
        final double SHOOTER_FF = 1 / 9.8;

        mTopPID.setP(SHOOTER_PID[0]);
        mTopPID.setI(SHOOTER_PID[1]);
        mTopPID.setD(SHOOTER_PID[2]);
        mTopPID.setFF(SHOOTER_FF);

        mBottomPID.setP(SHOOTER_PID[0]);
        mBottomPID.setI(SHOOTER_PID[1]);
        mBottomPID.setD(SHOOTER_PID[2]);
        mBottomPID.setFF(SHOOTER_FF);
        mBottomShooterMotor.burnFlash();
        mTopShooterMotor.burnFlash();
        mCollectorMotor.burnFlash();
        System.out.println("top motor faults: " + mTopShooterMotor.getFaults());
        System.out.println("top motor faults: " + mBottomShooterMotor.getFaults());

        Logger.recordMetadata("Scollector/MatchesState", "true");
    }

    public enum ScollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT, SHOOT, COLLECT_RAMP, COLLECT_AUTO_SHOOT, RAMP_SHOOTER, LOB, SHOOT_ASAP, COLLECT_RAMP_LOB,
        COLLECT_AUTO_LOB
    }

    @Override
    public void init() {
        mArmInPosition = false;
        mShooterTimer.reset();
        mShooterTimer.stop();
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("Scollector/MatchesState", matchesState());

        // mTopShooterVelocity.setDouble(mTopShooterMotor.getEncoder().getVelocity());
        // mBottomShooterVelocity.setDouble(mBottomShooterMotor.getEncoder().getVelocity());
        // mReadyToShootEntry.setBoolean(shooterAtSpeed());
        // mHasNoteEntry.setBoolean(hasNote());
    }

     @Override
    public boolean matchesState() {
        switch ((ScollectorState) getCurrentState()) {
            case IDLE:
                return mCollectorMotor.get() == 0;
            case COLLECT:
                return hasNote();
            case SPIT:
                return mCollectorMotor.get() == -1;
            case COLLECT_RAMP:
                return true;
            case COLLECT_AUTO_LOB:
            case COLLECT_AUTO_SHOOT:
            case SHOOT:
            case LOB:
            case SHOOT_ASAP:
                return !hasNote() && !noteCloseToShooter();
            case RAMP_SHOOTER:
                return true;
            default:
                return false;
        }
    }

    public void getToState() {

        switch ((ScollectorState) getCurrentState()) {
            case IDLE:
                mCollectorMotor.set(0);
                mTopShooterMotor.set(0);
                mBottomShooterMotor.set(0);
                break;
            case COLLECT:
                collect();
                mBottomShooterMotor.set(0);
                mTopShooterMotor.set(0);
                break;
            case COLLECT_RAMP:
                collect();
                driveMotors();
                break;
            case COLLECT_RAMP_LOB:
                collect();
                driveMotors(Constants.Scollector.MIN_SHOOTER_SPEED);
                break;
            case SPIT:
                mCollectorMotor.set(1);
                mTopPID.setReference(0, ControlType.kVelocity);
                break;
            case SHOOT:
                shoot();
                break;
            case COLLECT_AUTO_SHOOT:
                if (!hasNote()) {
                    collect();
                } else if (mArmInPosition && (noteCloseToShooter() || hasNote()) && shooterAtSpeed()) {
                    shoot();
                } else {
                    mCollectorMotor.set(0);
                }

                driveMotors();
                break;
            case COLLECT_AUTO_LOB:
                // if (!hasNote()) {
                //     collect();
                //     System.out.println("aaaaaaaaaaaa");
                // } else if (mArmInPosition && (noteCloseToShooter() || hasNote()) && shooterAtSpeed(ConfigMap.MIN_SHOOTER_SPEED)) {
                //     shoot();
                //     System.out.println("bbbbbbbbbbbbb");
                // } else {
                //     mCollectorMotor.set(0);
                //     System.out.println("cccccccccccccccccc: " + mArmInPosition + " :: " + noteCloseToShooter() + " :: " + hasNote() + " :: " + shooterAtSpeed(ConfigMap.MIN_SHOOTER_SPEED));
                // }

                driveMotors();
                break;
            case RAMP_SHOOTER:
                driveMotors();
                mCollectorMotor.set(0);
                break;
            case LOB:
                driveMotors(Constants.Scollector.MIN_SHOOTER_SPEED);
                break;
            case SHOOT_ASAP:
                if (mArmInPosition) {
                    shoot();
                } else {
                }

                driveMotors();
                break;
        }
    }

    public boolean hasNote() {
        double range = mBottomTof.getRange();
        if (!mBottomTof.isRangeValid() && range == 0.0) {
            return false;
        }

        return range <= 400;
    }

    public boolean noteCloseToShooter() {
        if (!mTopTof.isRangeValid() && mTopTof.getRange() == 0.0) {
            return false;
        }
        return mTopTof.getRange() <= 400;
    }

    public boolean shooterAtSpeed() {
        double leftSpeed = mBottomShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mTopShooterMotor.getEncoder().getVelocity();
        double leftDiff = Math.abs(leftSpeed - Constants.Scollector.MAX_SHOOTER_SPEED);
        double rightDiff = Math.abs(rightSpeed - Constants.Scollector.MAX_SHOOTER_SPEED);
        double point = Constants.Scollector.MAX_SHOOTER_SPEED - 1;
        final double TOLERANCE = 0.1;

        return leftSpeed >= point && rightSpeed >= point;

    }

    public boolean shooterAtSpeed(double speed) {
        double leftSpeed = mBottomShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mTopShooterMotor.getEncoder().getVelocity();
        double leftDiff = Math.abs(leftSpeed - Constants.Scollector.MAX_SHOOTER_SPEED);
        double rightDiff = Math.abs(rightSpeed - Constants.Scollector.MAX_SHOOTER_SPEED);
        double point = speed;
        final double TOLERANCE = 0.1;

        return leftSpeed >= point && rightSpeed >= point;

    }

    public void setArmReadyToShoot(boolean armReady) {
        mArmInPosition = armReady;
    }

    private void driveMotors() {
        double setpoint = Constants.Scollector.MAX_SHOOTER_SPEED;
        System.out.println(mTopPID.getOutputMax());
        mTopPID.setReference(setpoint, ControlType.kVelocity);
        mBottomPID.setReference(setpoint, ControlType.kVelocity);
    }

    private void driveMotors(double setPoint) {
        mTopPID.setReference(setPoint, ControlType.kVelocity);
        mBottomPID.setReference(setPoint, ControlType.kVelocity);
    }

    private void collect() {
        if (!hasNote() && !noteCloseToShooter()) {
            mCollectorMotor.set(-1);
        } else if (noteCloseToShooter()) {
            mCollectorMotor.set(0.25);
        } else if (hasNote()) {
            mCollectorMotor.set(0);
        }
    }

    private void shoot() {
        driveMotors();
        mCollectorMotor.set(-1);
    }



}
