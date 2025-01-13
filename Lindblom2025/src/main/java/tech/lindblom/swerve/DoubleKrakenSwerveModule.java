package tech.lindblom.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.SwerveModuleConfiguration;

public class DoubleKrakenSwerveModule {
    private String name;

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

    public DoubleKrakenSwerveModule(String name, SwerveModuleConstants constants) {
        this.name = name;
        driveMotor = new TalonFX(constants.DriveMotorId);
        steerMotor = new TalonFX(constants.SteerMotorId);
        steerEncoder = new CANcoder(constants.EncoderId);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveMotor.getConfigurator().apply(talonConfigs);

        StatusCode driveConfigStatusCode = driveMotor.getConfigurator().apply(talonConfigs);
        if (driveConfigStatusCode != StatusCode.OK) {
            System.err.println("COULD NOT CONFIGURE THE FOLLOWING MOTOR: " + constants.DriveMotorId);
        }


        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = constants.SteerMotorGains;

        talonConfigs.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap =
                true;

        talonConfigs.MotorOutput.Inverted =
                constants.SteerMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                //SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
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

        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

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

    public Rotation2d getAbsoluteAngle() {
        double reportedVal = steerEncoder.getAbsolutePosition().getValueAsDouble();

        reportedVal = reportedVal % 1.0;
        if(reportedVal < 0) {
            reportedVal += 1.0;
        }

        return new Rotation2d(reportedVal * 2 * Math.PI);
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * moduleConfiguration().metersPerRevolution;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    public void runDesiredModuleState(SwerveModuleState sentDesiredState) {
        sentDesiredState.optimize(getAbsoluteAngle());

        double angleToSetDeg = sentDesiredState.angle.getDegrees();
        steerMotor.setControl(new PositionVoltage(angleToSetDeg));
        double velocityToSet = sentDesiredState.speedMetersPerSecond;
        driveMotor.setControl(velocitySetter.withVelocity(velocityToSet));

        Logger.recordOutput("DriveModules/" + this.name + "/Optimized State", sentDesiredState);
        Logger.recordOutput("DriveModules/" + this.name + "/Requested Angle", angleToSetDeg);
        Logger.recordOutput("DriveModules/" + this.name + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("DriveModules/" + this.name + "/Turn Voltage", steerMotor.getMotorVoltage().getValueAsDouble()
        );
    }

    static SwerveModuleConfiguration moduleConfiguration() {
        SwerveModuleConfiguration ret_val = new SwerveModuleConfiguration();
        ret_val.metersPerRevolution = 1 / (0.1016 * Math.PI * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * .9766); //this is not named correctly
        ret_val.radiansPerRevolution = 2 * Math.PI * (14.0 / 50.0) * (10.0 / 60.0);
        ret_val.velocityConversion = ret_val.metersPerRevolution / 60.0;
        ret_val.radiansPerSecond = ret_val.radiansPerRevolution / 60.0;


        // SparkMax (NEO) / VORTEX?
        // Motor for pivot 1 motor
        // 2 motors for elevator
        // one inverted and follows
        ret_val.drivingP = 10;
        ret_val.drivingI = 0;
        ret_val.drivingD = 0;
        ret_val.drivingFF = 1.0 / (Constants.Drive.speedAt12Volts);
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
