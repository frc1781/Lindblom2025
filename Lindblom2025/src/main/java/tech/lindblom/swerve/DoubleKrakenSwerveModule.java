package tech.lindblom.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.SwerveModuleConfiguration;

public class DoubleKrakenSwerveModule extends SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final BaseStatusSignal[] signals;
    private double driveRotationsPerMeter = 0;

    private final PositionVoltage angleSetter = new PositionVoltage(0);
    private final VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);

    private final SwerveModulePosition internalState = new SwerveModulePosition();

    public DoubleKrakenSwerveModule(String name, int driveMotorID, int steerMotorId, int cancoderId, double cancoderOffset, boolean isInverted) {
        super(name, driveMotorID, steerMotorId, cancoderId, cancoderOffset);
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorId);
        steerEncoder = new CANcoder(cancoderId);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0.kP = moduleConfiguration().drivingP;
        talonConfigs.Slot0.kI = moduleConfiguration().drivingI;
        talonConfigs.Slot0.kD = moduleConfiguration().drivingD;
        talonConfigs.Slot0.kV = moduleConfiguration().drivingKV;
        talonConfigs.Slot0.kA = moduleConfiguration().drivingKA;
        talonConfigs.Slot0.kS = moduleConfiguration().drivingKS;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 400;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -400;
        driveMotor.getConfigurator().apply(talonConfigs);

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = new Slot0Configs();
        talonConfigs.Slot0.kP = moduleConfiguration().turningP;
        talonConfigs.Slot0.kI = moduleConfiguration().turningI;
        talonConfigs.Slot0.kD = moduleConfiguration().turningD;

        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = cancoderId;
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfigs.Feedback.SensorToMechanismRatio = moduleConfiguration().metersPerRevolution;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = (1 / (14.0 / 50.0) * (10.0 / 60.0)); // or this over (14.0 / 50.0) * (10.0 / 60.0)

        talonConfigs.ClosedLoopGeneral.ContinuousWrap =
                true; // Enable continuous wrap for swerve modules

        talonConfigs.MotorOutput.Inverted =
                isInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = cancoderOffset;
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
        cancoderConfigs.MagnetSensor.SensorDirection = 
            isInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        
        steerEncoder.getConfigurator().apply(cancoderConfigs); 

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = steerEncoder.getPosition();
        steerVelocity = steerEncoder.getVelocity();

        signals = new BaseStatusSignal[4];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
        signals[2] = steerPosition;
        signals[3] = steerVelocity;

        double rotationsPerWheelRotation = moduleConfiguration().radiansPerRevolution;
        double metersPerWheelRotation = moduleConfiguration().metersPerRevolution;
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    BaseStatusSignal[] getSignals() {
        return signals;
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        double reportedVal = steerEncoder.getAbsolutePosition().getValueAsDouble();

        reportedVal = reportedVal % 1.0;
        if(reportedVal < 0) {
            reportedVal += 1.0;
        }

        return new Rotation2d(reportedVal * 2 * Math.PI);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        drivePosition.refresh();
        driveVelocity.refresh();
        steerPosition.refresh();
        steerVelocity.refresh();

        Measure<AngleUnit> drive_rot =
                BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
        Measure<AngleUnit> angle_rot =
                BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);

        internalState.distanceMeters = drive_rot.baseUnitMagnitude() / driveRotationsPerMeter;
        internalState.angle = Rotation2d.fromRotations(angle_rot.baseUnitMagnitude());

        return internalState;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveVelocity.getValueAsDouble(), getAbsoluteAngle());
    }

    @Override
    public void runDesiredModuleState(SwerveModuleState sentDesiredState) {
        sentDesiredState.optimize(internalState.angle);
        Logger.recordOutput("DriveModules/" + this.name + "/Optimized State", sentDesiredState);

        double angleToSetDeg = sentDesiredState.angle.getRadians();
        Logger.recordOutput("DriveModules/" + this.name + "/Requested Angle", angleToSetDeg);
        steerMotor.setControl(angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = sentDesiredState.speedMetersPerSecond * driveRotationsPerMeter;
        Logger.recordOutput("DriveModules/" + this.name + "/Requested Velocity", velocityToSet);
        driveMotor.setControl(velocitySetter.withVelocity(velocityToSet).withFeedForward(moduleConfiguration().drivingFF));
        Logger.recordOutput("DriveModules/" + this.name + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("DriveModules/" + this.name + "/Turn Voltage", steerMotor.getMotorVoltage().getValueAsDouble()
        );
    }

    @Override
    void syncRelativeToAbsoluteEncoder() {

    }

    static SwerveModuleConfiguration moduleConfiguration() {
        SwerveModuleConfiguration ret_val = new SwerveModuleConfiguration();
        ret_val.metersPerRevolution = 1 / (0.1016 * Math.PI * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * .9766); //this is not named correctly
        ret_val.radiansPerRevolution = 2 * Math.PI * (14.0 / 50.0) * (10.0 / 60.0);
        ret_val.velocityConversion = ret_val.metersPerRevolution / 60.0;
        ret_val.radiansPerSecond = ret_val.radiansPerRevolution / 60.0;

        ret_val.drivingP = 10;
        ret_val.drivingI = 0;
        ret_val.drivingD = 0;
        ret_val.drivingFF = 1.0 / (Constants.Drive.MAX_VELOCITY_METERS_PER_SECOND);
        ret_val.drivingKS = 0.0154;
        ret_val.drivingKV = 0.2529;
        ret_val.drivingKA = 0.3;


        ret_val.turningP = 5;
        ret_val.turningI = 0.0;
        ret_val.turningD = 0.01;
        ret_val.turningFF = 0.0;

        ret_val.minDrivingMotorVoltage = -12;
        ret_val.maxDrivingMotorVoltage = 12;
        ret_val.minTurningMotorVoltage = -12;
        ret_val.maxTurningMotorVoltage = 12;

        return ret_val;
    }
}
