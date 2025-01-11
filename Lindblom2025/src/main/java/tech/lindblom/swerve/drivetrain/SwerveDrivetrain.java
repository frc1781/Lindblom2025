package tech.lindblom.swerve.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveDrivetrain<
        DriveMotorT extends CommonTalon,
        SteerMotorT extends CommonTalon,
        EncoderT extends ParentDevice
        > implements AutoCloseable {
    /**
     * Functional interface for device constructors.
     */
    @FunctionalInterface
    public static interface DeviceConstructor<DeviceT> {
        DeviceT create(int id, String canbus);
    }

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public static class SwerveDriveState implements Cloneable {
        /** The current pose of the robot */
        public Pose2d Pose = new Pose2d();
        /** The current velocity of the robot */
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        /** The current module states */
        public SwerveModuleState[] ModuleStates;
        /** The target module states */
        public SwerveModuleState[] ModuleTargets;
        /** The current module positions */
        public SwerveModulePosition[] ModulePositions;
        /** The raw heading of the robot, unaffected by vision updates and odometry resets */
        public Rotation2d RawHeading = new Rotation2d();
        /** The timestamp of the state capture, in the timebase of {@link Utils#getCurrentTimeSeconds()} */
        public double Timestamp;
        /** The measured odometry update period, in seconds */
        public double OdometryPeriod;
        /** Number of successful data acquisitions */
        public int SuccessfulDaqs;
        /** Number of failed data acquisitions */
        public int FailedDaqs;

        /**
         * Creates a deep copy of this state object.
         * This API is not thread-safe.
         */
        @Override
        public com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState clone() {
            final var toReturn = new com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState();
            toReturn.Pose = Pose;
            toReturn.Speeds = Speeds;
            toReturn.ModuleStates = ModuleStates.clone();
            toReturn.ModuleTargets = ModuleTargets.clone();
            toReturn.ModulePositions = ModulePositions.clone();
            toReturn.RawHeading = RawHeading;
            toReturn.Timestamp = Timestamp;
            toReturn.OdometryPeriod = OdometryPeriod;
            toReturn.SuccessfulDaqs = SuccessfulDaqs;
            toReturn.FailedDaqs = FailedDaqs;
            return toReturn;
        }

        protected void updateFromJni(SwerveJNI.DriveState driveState) {
            if (Pose.getX() != driveState.PoseX ||
                    Pose.getY() != driveState.PoseY ||
                    Pose.getRotation().getRadians() != driveState.PoseTheta)
            {
                Pose = new Pose2d(driveState.PoseX, driveState.PoseY, Rotation2d.fromRadians(driveState.PoseTheta));
            }

            if (Speeds.vxMetersPerSecond != driveState.SpeedsVx ||
                    Speeds.vyMetersPerSecond != driveState.SpeedsVy ||
                    Speeds.omegaRadiansPerSecond != driveState.SpeedsOmega)
            {
                Speeds = new ChassisSpeeds(driveState.SpeedsVx, driveState.SpeedsVy, driveState.SpeedsOmega);
            }

            for (int i = 0; i < ModuleStates.length; ++i) {
                if (ModuleStates[i].speedMetersPerSecond != driveState.ModuleStates[i].speed ||
                        ModuleStates[i].angle.getRadians() != driveState.ModuleStates[i].angle)
                {
                    ModuleStates[i] = new SwerveModuleState(driveState.ModuleStates[i].speed, Rotation2d.fromRadians(driveState.ModuleStates[i].angle));
                }
            }
            for (int i = 0; i < ModuleTargets.length; ++i) {
                if (ModuleTargets[i].speedMetersPerSecond != driveState.ModuleTargets[i].speed ||
                        ModuleTargets[i].angle.getRadians() != driveState.ModuleTargets[i].angle)
                {
                    ModuleTargets[i] = new SwerveModuleState(driveState.ModuleTargets[i].speed, Rotation2d.fromRadians(driveState.ModuleTargets[i].angle));
                }
            }
            for (int i = 0; i < ModulePositions.length; ++i) {
                if (ModulePositions[i].distanceMeters != driveState.ModulePositions[i].distance ||
                        ModulePositions[i].angle.getRadians() != driveState.ModulePositions[i].angle)
                {
                    ModulePositions[i] = new SwerveModulePosition(driveState.ModulePositions[i].distance, Rotation2d.fromRadians(driveState.ModulePositions[i].angle));
                }
            }

            if (RawHeading.getRadians() != driveState.RawHeading) {
                RawHeading = Rotation2d.fromRadians(driveState.RawHeading);
            }

            Timestamp = driveState.Timestamp;
            OdometryPeriod = driveState.OdometryPeriod;
            SuccessfulDaqs = driveState.SuccessfulDaqs;
            FailedDaqs = driveState.FailedDaqs;
        }
    }

    /**
     * Contains everything the control requests need to calculate the module state.
     */
    public static class SwerveControlParameters {
        /** ID of the native drivetrain instance, used for JNI calls */
        public int drivetrainId;

        /** The kinematics object used for control */
        public SwerveDriveKinematics kinematics;
        /** The locations of the swerve modules */
        public Translation2d[] moduleLocations;
        /** The max speed of the robot at 12 V output, in m/s */
        public double kMaxSpeedMps;

        /** The forward direction from the operator perspective */
        public Rotation2d operatorForwardDirection = new Rotation2d();
        /** The current chassis speeds of the robot */
        public ChassisSpeeds currentChassisSpeed = new ChassisSpeeds();
        /** The current pose of the robot */
        public Pose2d currentPose = new Pose2d();
        /** The timestamp of the current control apply, in the timebase of {@link Utils#getCurrentTimeSeconds()} */
        public double timestamp;
        /** The update period of control apply */
        public double updatePeriod;

        protected void updateFromJni(SwerveJNI.ControlParams controlParams) {
            kMaxSpeedMps = controlParams.kMaxSpeedMps;
            if (operatorForwardDirection.getRadians() != controlParams.operatorForwardDirection) {
                operatorForwardDirection = Rotation2d.fromRadians(controlParams.operatorForwardDirection);
            }
            currentChassisSpeed.vxMetersPerSecond = controlParams.currentChassisSpeedVx;
            currentChassisSpeed.vyMetersPerSecond = controlParams.currentChassisSpeedVy;
            currentChassisSpeed.omegaRadiansPerSecond = controlParams.currentChassisSpeedOmega;
            if (currentPose.getX() != controlParams.currentPoseX ||
                    currentPose.getY() != controlParams.currentPoseY ||
                    currentPose.getRotation().getRadians() != controlParams.currentPoseTheta)
            {
                currentPose = new Pose2d(
                        controlParams.currentPoseX,
                        controlParams.currentPoseY,
                        Rotation2d.fromRadians(controlParams.currentPoseTheta)
                );
            }
            timestamp = controlParams.timestamp;
            updatePeriod = controlParams.updatePeriod;
        }
    }

    /** Number of times to attempt config applies. */
    protected static final int kNumConfigAttempts = 2;

    /** ID of the native drivetrain instance, used for JNI calls. */
    protected final int m_drivetrainId;
    /** JNI instance to use for non-static JNI calls. This object is not thread-safe. */
    protected final SwerveJNI m_jni = new SwerveJNI();

    private final SwerveModule<DriveMotorT, SteerMotorT, EncoderT>[] m_modules;
    private final Translation2d[] m_moduleLocations;

    private final SwerveDriveKinematics m_kinematics;

    private final Pigeon2 m_pigeon2;
    private final SimSwerveDrivetrain m_simDrive;

    /** The control parameters supplied to a non-native SwerveRequest. */
    private final com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters m_controlParams = new com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters();
    /** The swerve request currently applied. */
    private SwerveRequest m_swerveRequest = new SwerveRequest.Idle();
    /** Handle to the native object used to apply a non-native SwerveRequest. */
    private long m_controlHandle = 0;

    /** JNI instance to use for telemetry JNI calls. This object is not thread-safe. */
    protected final SwerveJNI m_telemetryJNI;
    /** The telemetry function currently applied. */
    private Consumer<com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState> m_telemetryFunction = null;
    /** Handle to the native object used to call the telemetry function. */
    private long m_telemetryHandle = 0;

    /** Lock protecting the swerve drive state. */
    private final Lock m_stateLock = new ReentrantLock();
    /** The last received swerve drive state. */
    private final com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState m_cachedState = new com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState();
    /** The current operator forward direction. */
    private Rotation2d m_operatorForwardDirection = new Rotation2d();

    /** Performs swerve module updates in a separate thread to minimize latency. */
    public class OdometryThread {
        /**
         * Starts the odometry thread.
         */
        public final void start() {
            SwerveJNI.JNI_Odom_Start(m_drivetrainId);
        }

        /**
         * Stops the odometry thread.
         */
        public final void stop() {
            SwerveJNI.JNI_Odom_Stop(m_drivetrainId);
        }

        /**
         * Check if the odometry is currently valid.
         *
         * @return True if odometry is valid
         */
        public boolean isOdometryValid() {
            return SwerveJNI.JNI_IsOdometryValid(m_drivetrainId);
        }

        /**
         * Sets the odometry thread priority to a real time priority under the specified priority level
         *
         * @param priority Priority level to set the odometry thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher priority and 0 indicating lower priority.
         */
        public final void setThreadPriority(int priority) {
            SwerveJNI.JNI_Odom_SetThreadPriority(m_drivetrainId, priority);
        }
    }

    private final com.ctre.phoenix6.swerve.SwerveDrivetrain.OdometryThread m_odometryThread = new com.ctre.phoenix6.swerve.SwerveDrivetrain.OdometryThread();

    /** Thread responsible for disposing the native drivetrain on shutdown. */
    private Thread m_shutdownHook;

    private double m_lastErrorSeconds = 0.0;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param driveMotorConstructor Constructor for the drive motor, such as {@code TalonFX::new}
     * @param steerMotorConstructor Constructor for the steer motor, such as {@code TalonFX::new}
     * @param encoderConstructor    Constructor for the azimuth encoder, such as {@code CANcoder::new}
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public SwerveDrivetrain(
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<DriveMotorT> driveMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<SteerMotorT> steerMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<EncoderT> encoderConstructor,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        this(() -> {
                    long nativeDriveConstants = drivetrainConstants.createNativeInstance();
                    long nativeModuleConstants = SwerveModuleConstants.createNativeInstance(modules);

                    var drivetrain = SwerveJNI.JNI_CreateDrivetrain(nativeDriveConstants, nativeModuleConstants, modules.length);

                    SwerveJNI.JNI_DestroyConstants(nativeDriveConstants);
                    SwerveJNI.JNI_DestroyConstants(nativeModuleConstants);
                    return drivetrain;
                },
                driveMotorConstructor, steerMotorConstructor, encoderConstructor,
                drivetrainConstants, modules
        );
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param driveMotorConstructor   Constructor for the drive motor, such as {@code TalonFX::new}
     * @param steerMotorConstructor   Constructor for the steer motor, such as {@code TalonFX::new}
     * @param encoderConstructor      Constructor for the azimuth encoder, such as {@code CANcoder::new}
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public SwerveDrivetrain(
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<DriveMotorT> driveMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<SteerMotorT> steerMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<EncoderT> encoderConstructor,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        this(() -> {
                    long nativeDriveConstants = drivetrainConstants.createNativeInstance();
                    long nativeModuleConstants = SwerveModuleConstants.createNativeInstance(modules);

                    var drivetrain = SwerveJNI.JNI_CreateDrivetrainWithFreq(nativeDriveConstants, odometryUpdateFrequency,
                            nativeModuleConstants, modules.length);

                    SwerveJNI.JNI_DestroyConstants(nativeDriveConstants);
                    SwerveJNI.JNI_DestroyConstants(nativeModuleConstants);
                    return drivetrain;
                },
                driveMotorConstructor, steerMotorConstructor, encoderConstructor,
                drivetrainConstants, modules
        );
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param driveMotorConstructor     Constructor for the drive motor, such as {@code TalonFX::new}
     * @param steerMotorConstructor     Constructor for the steer motor, such as {@code TalonFX::new}
     * @param encoderConstructor        Constructor for the azimuth encoder, such as {@code CANcoder::new}
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public SwerveDrivetrain(
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<DriveMotorT> driveMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<SteerMotorT> steerMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<EncoderT> encoderConstructor,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        this(() -> {
                    long nativeDriveConstants = drivetrainConstants.createNativeInstance();
                    long nativeModuleConstants = SwerveModuleConstants.createNativeInstance(modules);

                    var drivetrain = SwerveJNI.JNI_CreateDrivetrainWithStddev(nativeDriveConstants, odometryUpdateFrequency,
                            odometryStandardDeviation.getData(), visionStandardDeviation.getData(),
                            nativeModuleConstants, modules.length);

                    SwerveJNI.JNI_DestroyConstants(nativeDriveConstants);
                    SwerveJNI.JNI_DestroyConstants(nativeModuleConstants);
                    return drivetrain;
                },
                driveMotorConstructor, steerMotorConstructor, encoderConstructor,
                drivetrainConstants, modules
        );
    }

    private SwerveDrivetrain(
            IntSupplier createNativeInst,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<DriveMotorT> driveMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<SteerMotorT> steerMotorConstructor,
            com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor<EncoderT> encoderConstructor,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        m_drivetrainId = createNativeInst.getAsInt();

        m_cachedState.ModuleStates = new SwerveModuleState[modules.length];
        m_cachedState.ModuleTargets = new SwerveModuleState[modules.length];
        m_cachedState.ModulePositions = new SwerveModulePosition[modules.length];
        m_jni.driveState.ModuleStates = new SwerveJNI.ModuleState[modules.length];
        m_jni.driveState.ModuleTargets = new SwerveJNI.ModuleState[modules.length];
        m_jni.driveState.ModulePositions = new SwerveJNI.ModulePosition[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            m_cachedState.ModuleStates[i] = new SwerveModuleState();
            m_cachedState.ModuleTargets[i] = new SwerveModuleState();
            m_cachedState.ModulePositions[i] = new SwerveModulePosition();
            m_jni.driveState.ModuleStates[i] = new SwerveJNI.ModuleState();
            m_jni.driveState.ModuleTargets[i] = new SwerveJNI.ModuleState();
            m_jni.driveState.ModulePositions[i] = new SwerveJNI.ModulePosition();
        }

        m_telemetryJNI = m_jni.clone();

        @SuppressWarnings("unchecked")
        var tmpModules = (SwerveModule<DriveMotorT, SteerMotorT, EncoderT>[])new SwerveModule<?, ?, ?>[modules.length];

        m_modules = tmpModules;
        m_moduleLocations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            m_modules[i] = new SwerveModule<DriveMotorT, SteerMotorT, EncoderT>(
                    driveMotorConstructor,
                    steerMotorConstructor,
                    encoderConstructor,
                    modules[i],
                    drivetrainConstants.CANBusName,
                    m_drivetrainId, i
            );
            m_moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
        }

        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);

        m_controlParams.drivetrainId = m_drivetrainId;
        m_controlParams.kinematics = m_kinematics;
        m_controlParams.moduleLocations = m_moduleLocations;

        m_pigeon2 = new Pigeon2(drivetrainConstants.Pigeon2Id, drivetrainConstants.CANBusName);
        m_simDrive = new SimSwerveDrivetrain(m_moduleLocations, m_pigeon2.getSimState(), modules);

        if (drivetrainConstants.Pigeon2Configs != null) {
            StatusCode retval = StatusCode.OK;
            for (int i = 0; i < kNumConfigAttempts; ++i) {
                retval = getPigeon2().getConfigurator().apply(drivetrainConstants.Pigeon2Configs);
                if (retval.isOK()) break;
            }
            if (!retval.isOK()) {
                System.out.println("Pigeon2 ID " + getPigeon2().getDeviceID() + " failed config with error: " + retval);
            }
        }
        /* do not start thread until after applying Pigeon 2 configs */
        m_odometryThread.start();

        m_shutdownHook = new Thread(() -> {
            m_shutdownHook = null;
            close();
        });
        Runtime.getRuntime().addShutdownHook(m_shutdownHook);
    }

    @Override
    public void close() {
        SwerveJNI.JNI_DestroyDrivetrain(m_drivetrainId);
        if (m_controlHandle != 0) {
            SwerveJNI.JNI_DestroyControl(m_controlHandle);
            m_controlHandle = 0;
        }
        if (m_telemetryHandle != 0) {
            SwerveJNI.JNI_DestroyTelemetry(m_telemetryHandle);
            m_telemetryHandle = 0;
        }

        if (m_shutdownHook != null) {
            Runtime.getRuntime().removeShutdownHook(m_shutdownHook);
            m_shutdownHook = null;
        }
    }

    /**
     * Updates all the simulation state variables for this
     * drivetrain class. User provides the update variables for the simulation.
     *
     * @param dtSeconds time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        m_simDrive.update(dtSeconds, supplyVoltage, m_modules);
    }

    /**
     * Gets whether the drivetrain is on a CAN FD bus.
     *
     * @return true if on CAN FD
     */
    public final boolean isOnCANFD() {
        return SwerveJNI.JNI_IsOnCANFD(m_drivetrainId);
    }

    /**
     * Gets the target odometry update frequency in Hz.
     *
     * @return Target odometry update frequency
     */
    public final double getOdometryFrequency() {
        return SwerveJNI.JNI_GetOdometryFrequency(m_drivetrainId);
    }

    /**
     * Gets the target odometry update frequency.
     *
     * @return Target odometry update frequency
     */
    public final Frequency getOdometryFrequencyMeasure() {
        return Hertz.of(getOdometryFrequency());
    }

    /**
     * Gets a reference to the odometry thread.
     *
     * @return Odometry thread
     */
    public final com.ctre.phoenix6.swerve.SwerveDrivetrain.OdometryThread getOdometryThread() {
        return m_odometryThread;
    }

    /**
     * Check if the odometry is currently valid.
     *
     * @return True if odometry is valid
     */
    public boolean isOdometryValid() {
        return SwerveJNI.JNI_IsOdometryValid(m_drivetrainId);
    }

    /**
     * Gets a reference to the kinematics used for the drivetrain.
     *
     * @return Swerve kinematics
     */
    public final SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request) {
        if (!m_modules[0].m_isValid) {
            double now = Utils.getCurrentTimeSeconds();
            if (now - m_lastErrorSeconds > 3.0) {
                m_lastErrorSeconds = now;
                DriverStation.reportError(
                        "[phoenix] SwerveDrivetrain configuration invalid, drive is disabled. Ensure that the SwerveDrivetrain and SwerveModuleConstants instances are constructed with matching motor types (TalonFX/TalonFXConfiguration, TalonFXS/TalonFXSConfiguration, etc.).",
                        true
                );
            }
            return;
        }

        if (m_swerveRequest != request) {
            m_swerveRequest = request;

            var prevControlHandle = m_controlHandle;

            if (request == null) {
                m_controlHandle = m_jni.JNI_SetControl(m_drivetrainId, null);
            } else if (request instanceof NativeSwerveRequest req) {
                req.applyNative(m_drivetrainId);
                m_controlHandle = 0;
            } else {
                m_controlHandle = m_jni.JNI_SetControl(m_drivetrainId, () -> {
                    m_controlParams.updateFromJni(m_jni.controlParams);
                    return request.apply(m_controlParams, m_modules).value;
                });
            }

            if (prevControlHandle != 0) {
                SwerveJNI.JNI_DestroyControl(prevControlHandle);
            }
        } else if (request instanceof NativeSwerveRequest req) {
            /* update the native object */
            req.applyNative(m_drivetrainId);
        }
    }

    /**
     * Gets the current state of the swerve drivetrain.
     * This includes information such as the pose estimate,
     * module states, and chassis speeds.
     *
     * @return Current state of the drivetrain
     */
    public final com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState getState() {
        m_jni.JNI_GetState(m_drivetrainId);
        try {
            m_stateLock.lock();
            m_cachedState.updateFromJni(m_jni.driveState);
            return m_cachedState;
        } finally {
            m_stateLock.unlock();
        }
    }

    /**
     * Gets a copy of the current state of the swerve drivetrain.
     * This includes information such as the pose estimate,
     * module states, and chassis speeds.
     * <p>
     * This can be used to get a thread-safe copy of the state object.
     *
     * @return Copy of the current state of the drivetrain
     */
    public final com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState getStateCopy() {
        m_jni.JNI_GetState(m_drivetrainId);
        try {
            m_stateLock.lock();
            m_cachedState.updateFromJni(m_jni.driveState);
            return m_cachedState.clone();
        } finally {
            m_stateLock.unlock();
        }
    }

    /**
     * Register the specified lambda to be executed whenever our SwerveDriveState function
     * is updated in our odometry thread.
     * <p>
     * It is imperative that this function is cheap, as it will be executed along with
     * the odometry call, and if this takes a long time, it may negatively impact the
     * odometry of this stack.
     * <p>
     * This can also be used for logging data if the function performs logging instead of telemetry.
     * Additionally, the SwerveDriveState object can be cloned and stored for later processing.
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerTelemetry(Consumer<com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState> telemetryFunction) {
        if (m_telemetryFunction != telemetryFunction) {
            m_telemetryFunction = telemetryFunction;

            var prevTelemetryHandle = m_telemetryHandle;

            if (telemetryFunction == null) {
                m_telemetryHandle = m_telemetryJNI.JNI_RegisterTelemetry(m_drivetrainId, null);
            } else {
                m_telemetryHandle = m_telemetryJNI.JNI_RegisterTelemetry(m_drivetrainId, () -> {
                    try {
                        m_stateLock.lock();
                        m_cachedState.updateFromJni(m_telemetryJNI.driveState);
                        telemetryFunction.accept(m_cachedState);
                    } finally {
                        m_stateLock.unlock();
                    }
                });
            }

            if (prevTelemetryHandle != 0) {
                SwerveJNI.JNI_DestroyTelemetry(prevTelemetryHandle);
            }
        }
    }

    /**
     * Configures the neutral mode to use for all modules' drive motors.
     *
     * @param neutralMode The drive motor neutral mode
     * @return Status code of the first failed config call, or OK if all succeeded
     */
    public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
        return StatusCode.valueOf(SwerveJNI.JNI_ConfigNeutralMode(m_drivetrainId, neutralMode.value));
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        SwerveJNI.JNI_TareEverything(m_drivetrainId);
    }

    /**
     * Resets the rotation of the robot pose to 0 from the
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}
     * perspective. This makes the current orientation of the robot X
     * forward for field-centric maneuvers.
     * <p>
     * This is equivalent to calling {@link #resetRotation} with the operator
     * perspective rotation.
     */
    public void seedFieldCentric() {
        SwerveJNI.JNI_SeedFieldCentric(m_drivetrainId);
    }

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     *
     * @param pose Pose to make the current pose
     */
    public void resetPose(Pose2d pose) {
        SwerveJNI.JNI_ResetPose(m_drivetrainId, pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    /**
     * Resets the translation of the robot pose without affecting rotation.
     * The translation should be from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance}
     * perspective.
     *
     * @param translation Translation to make the current translation
     */
    public void resetTranslation(Translation2d translation) {
        SwerveJNI.JNI_ResetTranslation(m_drivetrainId, translation.getX(), translation.getY());
    }

    /**
     * Resets the rotation of the robot pose without affecting translation.
     * The rotation should be from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance}
     * perspective.
     *
     * @param rotation Rotation to make the current rotation
     */
    public void resetRotation(Rotation2d rotation) {
        SwerveJNI.JNI_ResetRotation(m_drivetrainId, rotation.getRadians());
    }

    /**
     * Takes the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective direction
     * and treats it as the forward direction for
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     * <p>
     * This does not change the robot pose, which is in the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     * As a result, the robot pose may need to be reset using {@link #resetPose}.
     *
     * @param fieldDirection Heading indicating which direction is forward from
     *                       the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
        SwerveJNI.JNI_SetOperatorPerspectiveForward(m_drivetrainId, fieldDirection.getRadians());
    }

    /**
     * Returns the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective
     * direction that is treated as the forward direction for
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     *
     * @return Heading indicating which direction is forward from
     *         the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
     */
    public final Rotation2d getOperatorForwardDirection() {
        double operatorForwardDirectionRad = SwerveJNI.JNI_GetOperatorForwardDirection(m_drivetrainId);
        try {
            m_stateLock.lock();
            if (m_operatorForwardDirection.getRadians() != operatorForwardDirectionRad) {
                m_operatorForwardDirection = Rotation2d.fromRadians(operatorForwardDirectionRad);
            }
            return m_operatorForwardDirection;
        } finally {
            m_stateLock.unlock();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link SwerveDrivePoseEstimator#update} every loop.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that you must use a timestamp with an
     *                              epoch since system startup (i.e., the epoch of this
     *                              timestamp is the same epoch as
     *                              {@link Utils#getCurrentTimeSeconds}).
     *                              This means that you should use
     *                              {@link Utils#getCurrentTimeSeconds}
     *                              as your time source or sync the epochs.
     *                              An FPGA timestamp can be converted to the correct
     *                              timebase using {@link Utils#fpgaToCurrentTime}.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        SwerveJNI.JNI_AddVisionMeasurement(m_drivetrainId, visionRobotPoseMeters.getX(), visionRobotPoseMeters.getY(),
                visionRobotPoseMeters.getRotation().getRadians(), timestampSeconds);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link SwerveDrivePoseEstimator#update} every loop.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to {@link
     * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that you must use a timestamp with an
     *                                 epoch since system startup (i.e., the epoch of this
     *                                 timestamp is the same epoch as
     *                                 {@link Utils#getCurrentTimeSeconds}).
     *                                 This means that you should use
     *                                 {@link Utils#getCurrentTimeSeconds}
     *                                 as your time source or sync the epochs.
     *                                 An FPGA timestamp can be converted to the correct
     *                                 timebase using {@link Utils#fpgaToCurrentTime}.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs)
    {
        SwerveJNI.JNI_AddVisionMeasurementWithStdDev(m_drivetrainId, visionRobotPoseMeters.getX(), visionRobotPoseMeters.getY(),
                visionRobotPoseMeters.getRotation().getRadians(), timestampSeconds, visionMeasurementStdDevs.getData());
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these numbers to
     *                                 trust global measurements from vision less.
     *                                 This matrix is in the form [x, y, theta]ᵀ,
     *                                 with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        SwerveJNI.JNI_SetVisionMeasurementStdDevs(m_drivetrainId, visionMeasurementStdDevs.getData());
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp. Note that you must use a timestamp
     *                         with an epoch since system startup (i.e., the epoch of
     *                         this timestamp is the same epoch as
     *                         {@link Utils#getCurrentTimeSeconds}).
     *                         This means that you should use
     *                         {@link Utils#getCurrentTimeSeconds}
     *                         as your time source in this case.
     *                         An FPGA timestamp can be converted to the correct
     *                         timebase using {@link Utils#fpgaToCurrentTime}.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        double[] retval = SwerveJNI.JNI_SamplePoseAt(m_drivetrainId, timestampSeconds);
        if (retval == null) {
            return Optional.empty();
        }
        return Optional.of(new Pose2d(retval[0], retval[1], Rotation2d.fromRadians(retval[2])));
    }

    /**
     * Get a reference to the module at the specified index.
     * The index corresponds to the module described in the constructor.
     *
     * @param index Which module to get
     * @return Reference to SwerveModule
     */
    public final SwerveModule<DriveMotorT, SteerMotorT, EncoderT> getModule(int index) {
        if (index >= m_modules.length) {
            return null;
        }
        return m_modules[index];
    }

    /**
     * Get a reference to the full array of modules.
     * The indexes correspond to the module described in the constructor.
     *
     * @return Reference to the SwerveModule array
     */
    public final SwerveModule<DriveMotorT, SteerMotorT, EncoderT>[] getModules() {
        return m_modules;
    }

    /**
     * Gets the locations of the swerve modules.
     *
     * @return Reference to the array of swerve module locations
     */
    public final Translation2d[] getModuleLocations() {
        return m_moduleLocations;
    }

    /**
     * Gets the current orientation of the robot as a {@link Rotation3d} from
     * the Pigeon 2 quaternion values.
     *
     * @return The robot orientation as a {@link Rotation3d}
     */
    public final Rotation3d getRotation3d() {
        return m_pigeon2.getRotation3d();
    }

    /**
     * Gets this drivetrain's Pigeon 2 reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This drivetrain's Pigeon 2 reference
     */
    public final Pigeon2 getPigeon2() {
        return m_pigeon2;
    }
}
