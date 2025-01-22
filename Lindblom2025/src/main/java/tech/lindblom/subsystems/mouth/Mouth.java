package tech.lindblom.subsystems.mouth;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import java.util.HashMap;


public class Mouth extends StateSubsystem{
    private final SparkMax spinMotor;
    private final SparkMax positionMotor;

    private final HashMap<MouthState, Double> positions = new HashMap<>();

    public Mouth() [
        super("Mouth", MouthState.UP);

        //Spin Motor
        spinMotor = new SparkMax(Constants.Mouth.SPIN_MOUTH_MOTOR, MotorType.kBrushless);

        SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
        spinMotorConfig.idleMode(IdleMode.kCoast);
        spinMotorConfig.smartCurrentLimit(30);
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Position Motor
        positionMotor = new SparkMax(Constants.Mouth.POSITION_MOUTH_MOTOR, MotorType.kBrushless);
        SparkMaxConfig positionMotorConfig = new SparkMaxConfig();
        positionMotorConfig.idleMode(IdleMode.kCoast);
        positionMotorConfig.smartCurrentLimit(30);
        positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(MouthState.UP, 0);
        positions.put(MouthState.DOWN, 0);
    ]

    @Override
    public boolean matchesState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'matchesState'");
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }
    
    public enum MouthState implements SubsystemState {
        UP,
        DOWN
    }
}
