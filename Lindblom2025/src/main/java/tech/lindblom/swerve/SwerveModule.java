package tech.lindblom.swerve;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import tech.lindblom.utils.SwerveModuleConfiguration;

public abstract class SwerveModule {

    public final String name;

    public SwerveModule(String _name, int driveMotorID, int turnMotorID, int cancoderId, double cancoderOffset) {
        name = _name;
    }

    public SwerveModule(int driveMotorID, int turnMotorID, int cancoderId, double cancoderOffset) {
        name = "Unnamed Swerve Module";
    }

    protected SwerveModuleState desiredState; 

    public void init() {
        
    }

    public abstract Rotation2d getAbsoluteAngle();

    public abstract SwerveModulePosition getModulePosition();

    public abstract SwerveModuleState getCurrentState();

    public abstract void runDesiredModuleState(SwerveModuleState sentDesiredState);

    abstract void syncRelativeToAbsoluteEncoder();

    static SwerveModuleConfiguration moduleConfiguration() {
        throw new UnsupportedOperationException();
    }

    private CANcoderConfiguration absoluteEncoderConfiguration(double offset) {
        throw new UnsupportedOperationException();
    }
}