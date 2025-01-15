package tech.lindblom.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.SwerveModuleConfiguration;

public class DoubleKrakenSwerveModule extends SwerveModule {
    private final TalonFX mDriveMotor;
    private final TalonFX mTurnMotor;

    private final CANcoder mTurnAbsoluteEncoder;
    private boolean isInverted;

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward (
            moduleConfiguration().drivingKS,
            moduleConfiguration().drivingKV,
            moduleConfiguration().drivingKA
    );

    private final ProfiledPIDController turningController = new ProfiledPIDController(
            moduleConfiguration().turningP,
            moduleConfiguration().turningI,
            moduleConfiguration().turningD,
            new TrapezoidProfile.Constraints(
                    moduleConfiguration().minTurningMotorVoltage, moduleConfiguration().maxTurningMotorVoltage
            )
    );


    public DoubleKrakenSwerveModule(String name, int driveMotorID, int turnMotorID, int cancoderID, double cancoderOffset, boolean inverted) {
        super(name, driveMotorID, turnMotorID, cancoderID, cancoderOffset);
        this.isInverted = inverted;

        mDriveMotor = new TalonFX(driveMotorID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = moduleConfiguration().metersPerRevolution; // not sure
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 35;
        driveConfig.CurrentLimits.withSupplyCurrentLimit(60);
        driveConfig.CurrentLimits.withSupplyCurrentLowerTime(0.1);
        driveConfig.Slot0.kP = moduleConfiguration().drivingP; //need to be tuned
        driveConfig.Slot0.kI = moduleConfiguration().drivingI;
        driveConfig.Slot0.kD = moduleConfiguration().drivingD;
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;  //need to be investigated
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        mDriveMotor.getConfigurator().apply(driveConfig);

        mTurnAbsoluteEncoder = new CANcoder(cancoderID);

        StatusCode statusCode = mTurnAbsoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration(cancoderOffset));
        if(statusCode != StatusCode.OK) {
            DriverStation.reportError("Could not configure CANcoder with ID: " + cancoderID, false);
        }

        mTurnMotor = new TalonFX(turnMotorID);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        if (inverted) {
            turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 35;
        turnConfig.CurrentLimits.withSupplyCurrentLimit(60);
        turnConfig.CurrentLimits.withSupplyCurrentLowerTime(0.1);
        turnConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        mTurnMotor.getConfigurator().apply(turnConfig);
        turningController.enableContinuousInput(0.999755859375, -1);
        turningController.reset(getAbsoluteRotation());

        Logger.recordOutput("DriveModule/" + name + "/Offset", cancoderOffset);
        Logger.recordOutput("DriveModule/" + name + "/Offset", cancoderOffset);

        Logger.recordOutput("DriveModule/" + name + "/Drive Motor Velocity", 0.0);
        Logger.recordOutput("DriveModule/" + name + "/Drive Motor Position", 0.0);
        Logger.recordOutput("DriveModule/" + name + "/Turning Motor Position", 0.0);
        Logger.recordOutput("DriveModule/" + name + "/CANCoder Position", 0.0);
        Logger.recordOutput("DriveModule/" + name + "/Turning Motor CANCoder Difference", 0.0);

        Logger.recordOutput("DriveModule/" + name + "/Drive Requested Velocity", 0.0);
        Logger.recordOutput("DriveModule/" + name + "/Turn Requested Position", 0.0);
    }

    public double getAbsoluteRotation() {
        return mTurnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public SwerveModuleState getCurrentState() {
        Logger.recordOutput("DriveModule/" + this.name + "/Turn CAN" ,getAbsoluteAngle());
        return new SwerveModuleState(getDriveMotorSpeed(), getAbsoluteAngle());
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(getAbsoluteRotation());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveMotorPosition(), getAbsoluteAngle());
    }

    public void runDesiredModuleState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAbsoluteAngle());
        Logger.recordOutput("DriveModule/" + this.name + "/Drive Requested Velocity", optimizedState.speedMetersPerSecond);
        Logger.recordOutput("DriveModule/" + this.name + "/Turn Requested Position", optimizedState.angle.getRotations());
        Logger.recordOutput("DriveModule/" + this.name + "/RequestedAndRealDifference", Math.abs(optimizedState.angle.getRotations() - getAbsoluteRotation()));

        if (Math.abs(optimizedState.angle.getRotations() - getAbsoluteRotation()) > 0.01) {
            double turningControllerOutput = turningController.calculate(getAbsoluteRotation(), optimizedState.angle.getRotations());
            Logger.recordOutput("DriveModule/" + this.name + "/PID", turningControllerOutput);
            mTurnMotor.set(turningControllerOutput);
        }

        double FF = driveFF.calculate(optimizedState.speedMetersPerSecond);

        Logger.recordOutput("DriveModule/" + this.name + "/FeedForwardOutput", FF);

        mDriveMotor.set(FF);

        Logger.recordOutput("DriveModule/" + this.name + "/Drive Motor Velocity", getDriveMotorSpeed());
        Logger.recordOutput("DriveModule/" + this.name + "/Drive Motor Position", getDriveMotorPosition());
        Logger.recordOutput("DriveModule/" + this.name + "/Turning Motor Position", mTurnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private double getDriveMotorSpeed() {
        return mDriveMotor.getVelocity().getValueAsDouble();
    }

    private double getDriveMotorPosition() {
        return mDriveMotor.getPosition().getValueAsDouble();
    }

    @Override
    void syncRelativeToAbsoluteEncoder() {
        return;
    }

    static SwerveModuleConfiguration moduleConfiguration() {
        SwerveModuleConfiguration ret_val = new SwerveModuleConfiguration();
        ret_val.metersPerRevolution = 1 / (0.1016 * Math.PI * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * .9766); //this is not named correctly
        ret_val.radiansPerRevolution = 2 * Math.PI * (14.0 / 50.0) * (10.0 / 60.0);
        ret_val.velocityConversion = ret_val.metersPerRevolution / 60.0;
        ret_val.radiansPerSecond = ret_val.radiansPerRevolution / 60.0;

        ret_val.drivingP = 0;
        ret_val.drivingI = 0;
        ret_val.drivingD = 0;
        ret_val.drivingFF = 1.0 / (Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND);
        ret_val.drivingKS = 0.0154;
        ret_val.drivingKV = 0.2529;
        ret_val.drivingKA = 0.3;

        ret_val.turningP = 0.1;
        ret_val.turningI = 0;
        ret_val.turningD = 0;
        ret_val.turningFF = 0.0;

        ret_val.minDrivingMotorVoltage = -1;
        ret_val.maxDrivingMotorVoltage = 1;
        ret_val.minTurningMotorVoltage = -1;
        ret_val.maxTurningMotorVoltage = 1;

        return ret_val;
    }

    CANcoderConfiguration absoluteEncoderConfiguration(double magnetOffset) {
        CANcoderConfiguration ret_val = new CANcoderConfiguration();

        ret_val.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
        ret_val.MagnetSensor.MagnetOffset = magnetOffset;
        if (isInverted) {
            ret_val.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        } else {
            ret_val.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }

        return ret_val;
    }
}
