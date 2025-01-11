package tech.lindblom.swerve.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;

/**
 * Common constants for a swerve drivetrain.
 */
public class SwerveDrivetrainConstants {
    /**
     * Name of the CAN bus the swerve drive is on. Possible CAN bus strings are:
     *
     * <ul>
     *   <li> empty string or "rio" for the native roboRIO CAN bus
     *   <li> CANivore name or serial number
     *   <li> "*" for any CANivore seen by the program
     * </ul>
     *
     * Note that all devices must be on the same CAN bus.
     */
    public String CANBusName = "rio";
    /**
     * CAN ID of the Pigeon2 on the drivetrain.
     */
    public int Pigeon2Id = 0;
    /**
     * The configuration object to apply to the Pigeon2. This defaults to null. If
     * this remains null, then the Pigeon2 will not be configured (and whatever
     * configs are on it remain on it). If this is not null, the Pigeon2 will be
     * overwritten with these configs.
     */
    public Pigeon2Configuration Pigeon2Configs = null;

    /**
     * Modifies the CANBusName parameter and returns itself.
     * <p>
     * Name of the CAN bus the swerve drive is on. Possible CAN bus strings are:
     *
     * <ul>
     *   <li> empty string or "rio" for the native roboRIO CAN bus
     *   <li> CANivore name or serial number
     *   <li> "*" for any CANivore seen by the program
     * </ul>
     *
     * Note that all devices must be on the same CAN bus.
     *
     * @param newCANBusName Parameter to modify
     * @return this object
     */
    public com.ctre.phoenix6.swerve.SwerveDrivetrainConstants withCANBusName(String newCANBusName) {
        this.CANBusName = newCANBusName;
        return this;
    }

    /**
     * Modifies the Pigeon2Id parameter and returns itself.
     * <p>
     * CAN ID of the Pigeon2 on the drivetrain.
     *
     * @param newPigeon2Id Parameter to modify
     * @return this object
     */
    public com.ctre.phoenix6.swerve.SwerveDrivetrainConstants withPigeon2Id(int newPigeon2Id) {
        this.Pigeon2Id = newPigeon2Id;
        return this;
    }

    /**
     * Modifies the Pigeon2Configs parameter and returns itself.
     * <p>
     * The configuration object to apply to the Pigeon2. This defaults to null. If
     * this remains null, then the Pigeon2 will not be configured (and whatever
     * configs are on it remain on it). If this is not null, the Pigeon2 will be
     * overwritten with these configs.
     *
     * @param newPigeon2Configs Parameter to modify
     * @return this object
     */
    public com.ctre.phoenix6.swerve.SwerveDrivetrainConstants withPigeon2Configs(Pigeon2Configuration newPigeon2Configs) {
        this.Pigeon2Configs = newPigeon2Configs;
        return this;
    }

    long createNativeInstance() {
        return SwerveJNI.JNI_CreateDrivetrainConstants(
                CANBusName,
                Pigeon2Id
        );
    }
}
