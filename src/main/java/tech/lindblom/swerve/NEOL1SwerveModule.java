package tech.lindblom.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import tech.team1781.ConfigMap;
import tech.team1781.utils.SwerveModuleConfiguration;

public class NEOL1SwerveModule extends SwerveModule{
    private SwerveModuleState mDesiredState = new SwerveModuleState();

    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final SparkPIDController mDrivePID;
    private final SparkPIDController mTurnPID;
    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mTurnEncoder;
    private final CANcoder mTurnAbsoluteEncoder;
    private final int mCancoderID;
    private final double mCancoderOffset;

    public NEOL1SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, double cancoderOffset) {
        super(driveMotorID, turnMotorID, cancoderID, cancoderOffset);
        mCancoderID = cancoderID;
        mCancoderOffset = cancoderOffset;
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mTurnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        
        mDriveMotor.restoreFactoryDefaults();
        mTurnMotor.restoreFactoryDefaults();
        mTurnMotor.setInverted(true);
        mDriveMotor.setInverted(true);
        mDrivePID = mDriveMotor.getPIDController();
        mTurnPID = mTurnMotor.getPIDController();
        mTurnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        mTurnPID.setPositionPIDWrappingMinInput(0);
        mTurnPID.setPositionPIDWrappingEnabled(true);
        mDriveEncoder = mDriveMotor.getEncoder();
        mTurnEncoder = mTurnMotor.getEncoder();
        
        mDrivePID.setFeedbackDevice(mDriveEncoder);
        mTurnPID.setFeedbackDevice(mTurnEncoder);

        mTurnAbsoluteEncoder = new CANcoder(cancoderID);
        StatusCode statusCode = mTurnAbsoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration(cancoderOffset));
        if(statusCode != StatusCode.OK) {
            DriverStation.reportError("Could not configure CANcoder with ID: " + cancoderID, false);
        }

        mDrivePID.setP(moduleConfiguration().drivingP);
        mDrivePID.setI(moduleConfiguration().drivingI);
        mDrivePID.setD(moduleConfiguration().drivingD);
        mDrivePID.setFF(moduleConfiguration().drivingFF);

        mTurnPID.setP(moduleConfiguration().turningP);
        mTurnPID.setI(moduleConfiguration().turningI);
        mTurnPID.setD(moduleConfiguration().turningD);


        mTurnEncoder.setPositionConversionFactor(moduleConfiguration().radiansPerRevolution);
        mTurnEncoder.setVelocityConversionFactor(moduleConfiguration().radiansPerSecond);

        mDriveEncoder.setPositionConversionFactor(moduleConfiguration().metersPerRevolution);
        mDriveEncoder.setVelocityConversionFactor(moduleConfiguration().velocityConversion);
        mTurnPID.setFF(moduleConfiguration().turningFF);

        mDrivePID.setOutputRange(moduleConfiguration().minDrivingMotorVoltage, moduleConfiguration().maxDrivingMotorVoltage);
        mTurnPID.setOutputRange(moduleConfiguration().minTurningMotorVoltage, moduleConfiguration().maxTurningMotorVoltage);

        mDriveMotor.burnFlash();
        mTurnMotor.burnFlash();
        mTurnEncoder.setPosition(getAbsoluteAngle().getRadians());
        mDriveEncoder.setPosition(0);
    }

    public Rotation2d getAbsoluteAngle() {
        double reportedVal = mTurnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        reportedVal += mCancoderOffset; //add offset instead of setting in code
        reportedVal = reportedVal % 1.0;
        if(reportedVal < 0) {
            reportedVal += 1.0;
        }

        return new Rotation2d(reportedVal * 2 * Math.PI);
    }

    @Override 
    public void init() {
        // mTurnEncoder.setPosition(getAbsoluteAngle().getRadians());
        syncRelativeToAbsoluteEncoder();
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(mDriveEncoder.getVelocity(), getAbsoluteAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(mDriveEncoder.getPosition(), getAbsoluteAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(mTurnEncoder.getPosition()));
        mDrivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        mTurnPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
        
        syncRelativeToAbsoluteEncoder();
    }

    public void syncRelativeToAbsoluteEncoder() {
        double modAbs = getAbsoluteAngle().getRadians() % (Math.PI * 2);
        double modRel = mTurnEncoder.getPosition() % (Math.PI * 2);
        
        if(modAbs < 0) 
            modAbs += Math.PI * 2;
        if(modRel < 0) 
            modRel += Math.PI * 2;
        
        double diff = modAbs - modRel;
        if(Math.abs(diff) > 1) {
            System.out.printf("synching %d with abs %.2f rel %.2f\n", mCancoderID, modAbs, modRel);
            mTurnEncoder.setPosition(getAbsoluteAngle().getRadians());
        }
    }

    public void printModuleState() {
        System.out.println("===============================================");
        System.out.printf("Module %d:\n", mCancoderID);
        System.out.printf("  abs: %.2f\n", getAbsoluteAngle().getDegrees());
        System.out.printf("  rel: %.2f\n", mTurnEncoder.getPosition() * 360 / (Math.PI * 2));
        System.out.println("===============================================");
    }

    static SwerveModuleConfiguration moduleConfiguration() {
        SwerveModuleConfiguration ret_val = new SwerveModuleConfiguration();

        ret_val.metersPerRevolution = 0.102 * Math.PI * (14.0/50.0) * (25.0/19.0) * (15.0 / 45.0);
        ret_val.radiansPerRevolution = 2 * Math.PI * (14.0/50.0) * (10.0/60.0);
        ret_val.velocityConversion = ret_val.metersPerRevolution / 60;
        ret_val.radiansPerSecond = ret_val.radiansPerRevolution / 60.0;

        ret_val.drivingP = 0.1;
        ret_val.drivingI = 0.0;
        ret_val.drivingD = 0.01;
        ret_val.drivingFF = 1.0 / (ConfigMap.MAX_VELOCITY_METERS_PER_SECOND + 0.08);

        ret_val.turningP = 1; 
        ret_val.turningI = 0.0;
        ret_val.turningD = 0.01; 
        ret_val.turningFF = 0.0;

        ret_val.minDrivingMotorVoltage = -1;
        ret_val.maxDrivingMotorVoltage = 1;
        ret_val.minTurningMotorVoltage = -1;
        ret_val.maxTurningMotorVoltage = 1;

        return ret_val;
    }

    private CANcoderConfiguration absoluteEncoderConfiguration(double magnetOffset) {
        CANcoderConfiguration ret_val = new CANcoderConfiguration(); 
        System.out.println("abs encoder config id: " + mCancoderID + " magOffset: " + magnetOffset);
        ret_val.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        ret_val.MagnetSensor.MagnetOffset = 0.0; //magnetOffset;
        ret_val.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return ret_val;
    }
}
