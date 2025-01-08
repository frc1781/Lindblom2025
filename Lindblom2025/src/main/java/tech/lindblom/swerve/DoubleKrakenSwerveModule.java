package tech.lindblom.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.SwerveModuleConfiguration;

public class DoubleKrakenSwerveModule extends SwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder cancoder;

    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Angle> steerPosition;
    private StatusSignal<AngularVelocity> steerVelocity;
    private BaseStatusSignal[] signals;
    private double driveRotationsPerMeter = 0;

    private PositionVoltage angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition internalState = new SwerveModulePosition();

    public DoubleKrakenSwerveModule(String _name, int driveMotorID, int turnMotorID, int cancoderId, double cancoderOffset) {
        super(_name, driveMotorID, turnMotorID, cancoderId, cancoderOffset);

        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(turnMotorID);
        cancoder = new CANcoder(cancoderId);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;

        driveMotor.getConfigurator().apply(talonConfigs);

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = cancoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        talonConfigs.MotorOutput.Inverted =   InvertedValue.Clockwise_Positive;
        steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = cancoderOffset;
        cancoder.getConfigurator().apply(cancoderConfigs);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = cancoder.getPosition();
        steerVelocity = cancoder.getVelocity();

        signals = new BaseStatusSignal[4];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
        signals[2] = steerPosition;
        signals[3] = steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return null;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        /* Refresh all signals */
        drivePosition.refresh();
        driveVelocity.refresh();
        steerPosition.refresh();

        /* Now latency-compensate our signals */
        double drive_rot =
                BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity).baseUnitMagnitude();
        double angle_rot =
                BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity).baseUnitMagnitude();

        /* And push them into a SwerveModuleState object to return */
        internalState.distanceMeters = drive_rot / driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        internalState.angle = Rotation2d.fromRotations(angle_rot);

        return internalState;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveVelocity.getValueAsDouble(), new Rotation2d(steerPosition.getValue()));
    }

    @Override
    public void runDesiredModuleState(SwerveModuleState state) {
        state.optimize(internalState.angle);

        double angleToSetDeg = state.angle.getRotations();
        steerMotor.setControl(angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = state.speedMetersPerSecond * driveRotationsPerMeter;
        driveMotor.setControl(velocitySetter.withVelocity(velocityToSet));
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

    @Override
    void syncRelativeToAbsoluteEncoder() {
        return;
    }
}
