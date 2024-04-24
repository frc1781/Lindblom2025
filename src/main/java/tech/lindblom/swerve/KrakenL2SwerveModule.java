package tech.lindblom.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import tech.lindblom.Config;

public class KrakenL2SwerveModule extends SwerveModule { 
    private SwerveModuleState mDesiredState = new SwerveModuleState();
    private final TalonFX mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final SparkPIDController mTurnPID;
    private final VelocityVoltage mDriveVelocity = new VelocityVoltage(0);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward (
        moduleConfiguration().drivingKS,
        moduleConfiguration().drivingKV,
        moduleConfiguration().drivingKA
    ); 
    private final RelativeEncoder mTurnEncoder;
    private final CANcoder mTurnAbsoluteEncoder;

    public KrakenL2SwerveModule(String name,int driveMotorID, int turnMotorID, int cancoderID, double cancoderOffset) {
        super(name, driveMotorID, turnMotorID, cancoderID, cancoderOffset);

        //DRIVE MOTOR CREATE AND CONFIGURE 
        mDriveMotor = new TalonFX(driveMotorID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Gear Ratio Config: using metersPerRevolution but not sure if that is correct here, should be 
        //gear ratio
        driveConfig.Feedback.SensorToMechanismRatio = moduleConfiguration().metersPerRevolution; // not sure
        /* Current Limiting */
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 35;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        driveConfig.Slot0.kP = moduleConfiguration().drivingP; //need to be tuned
        driveConfig.Slot0.kI = moduleConfiguration().drivingI;
        driveConfig.Slot0.kD = moduleConfiguration().drivingD;
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;  //need to be investigated
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0; 
        
        mDriveMotor.getConfigurator().apply(driveConfig);

        mTurnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        
        mTurnMotor.restoreFactoryDefaults();
        mTurnMotor.setSmartCurrentLimit(30);
        mTurnMotor.setInverted(true);

        mTurnPID = mTurnMotor.getPIDController();
        mTurnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        mTurnPID.setPositionPIDWrappingMinInput(0);
        mTurnPID.setPositionPIDWrappingEnabled(true);
        mTurnEncoder = mTurnMotor.getEncoder();

        
        mTurnPID.setFeedbackDevice(mTurnEncoder);

        mTurnAbsoluteEncoder = new CANcoder(cancoderID);
        StatusCode statusCode = mTurnAbsoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration(cancoderOffset));
        if(statusCode != StatusCode.OK) {
            DriverStation.reportError("Could not configure CANcoder with ID: " + cancoderID, false);
        }

        mTurnPID.setP(moduleConfiguration().turningP);
        mTurnPID.setI(moduleConfiguration().turningI);
        mTurnPID.setD(moduleConfiguration().turningD);
        mTurnPID.setFF(moduleConfiguration().turningFF);

        mTurnEncoder.setPositionConversionFactor(moduleConfiguration().radiansPerRevolution);
        mTurnEncoder.setVelocityConversionFactor(moduleConfiguration().radiansPerSecond);

        mTurnPID.setFF(moduleConfiguration().turningFF);

        mTurnPID.setOutputRange(moduleConfiguration().minTurningMotorVoltage, moduleConfiguration().maxTurningMotorVoltage);
        mTurnEncoder.setPosition(getAbsoluteAngle().getRadians());

        mTurnMotor.burnFlash();
    }

    public Rotation2d getAbsoluteAngle() {
        double reportedVal = mTurnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();

        reportedVal = reportedVal % 1.0;
        if(reportedVal < 0) {
            reportedVal += 1.0;
        }

        return new Rotation2d(reportedVal * 2 * Math.PI);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotorSpeed(), getAbsoluteAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveMotorPosition(), getAbsoluteAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAbsoluteAngle());
        mTurnPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
        // mDriveVelocity.Velocity = desiredState.speedMetersPerSecond;
        // mDriveVelocity.FeedForward = driveFF.calculate(desiredState.speedMetersPerSecond);
        mDriveMotor.set(optimizedState.speedMetersPerSecond / Config.MAX_VELOCITY_METERS_PER_SECOND);
        mDesiredState = optimizedState;
        syncRelativeToAbsoluteEncoder();
    }

    public void printDriveMotor() {
        // System.out.print("," + mDesiredState.speedMetersPerSecond / ConfigMap.MAX_VELOCITY_METERS_PER_SECOND + "\n");
    }

    private double getDriveMotorSpeed() {
        return mDriveMotor.getVelocity().getValueAsDouble();
    }

    private double getDriveMotorPosition() {
        return mDriveMotor.getPosition().getValueAsDouble();
    }

    void syncRelativeToAbsoluteEncoder() {
        if(mTurnEncoder.getVelocity() >= 0.5) {
            return;
        }

        double turnEncoderPosition = mTurnEncoder.getPosition();
        double diff = getAbsoluteAngle().getRadians() - turnEncoderPosition;

        if(Math.abs(diff) > 0.02) {
            mTurnEncoder.setPosition(getAbsoluteAngle().getRadians());
        }

    }

    static SwerveModuleConfiguration moduleConfiguration() {
        SwerveModuleConfiguration ret_val = new SwerveModuleConfiguration();

        ret_val.metersPerRevolution = 1 / (0.1016 * Math.PI * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)); //this is not named correctly
        ret_val.radiansPerRevolution = 2 * Math.PI * (14.0 / 50.0) * (10.0 / 60.0);
        ret_val.velocityConversion = ret_val.metersPerRevolution / 60.0;
        ret_val.radiansPerSecond = ret_val.radiansPerRevolution / 60.0;

        //MUST TUNE ALL OF THESE AND DO SYSID 
        ret_val.drivingP = 0.1; 
        ret_val.drivingI = 0.0;
        ret_val.drivingD = 0.01;
        ret_val.drivingFF = 1.0 / (Config.MAX_VELOCITY_METERS_PER_SECOND + 0.08);
        ret_val.drivingKS = 0.02;
        ret_val.drivingKV = 1.0 / (Config.MAX_VELOCITY_RADIANS_PER_SECOND + 0.08) - ret_val.drivingKS;
        ret_val.drivingKA = 5.4; 

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

    static CANcoderConfiguration absoluteEncoderConfiguration(double magnetOffset) {
        CANcoderConfiguration ret_val = new CANcoderConfiguration(); 

        ret_val.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        ret_val.MagnetSensor.MagnetOffset = magnetOffset;
        ret_val.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return ret_val;
    }
}
