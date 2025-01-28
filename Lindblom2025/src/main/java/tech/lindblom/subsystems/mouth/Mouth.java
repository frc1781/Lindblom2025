package tech.lindblom.subsystems.mouth;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

import java.util.HashMap;


public class Mouth extends StateSubsystem {
    private final SparkMax spinMotor;
    private final SparkMax positionMotor;
    private final SparkClosedLoopController motionController;

    private final HashMap<MouthState, Double> positions = new HashMap<>();

    public Mouth() {
        super("Mouth", MouthState.UP);

        //Spin Motor
        spinMotor = new SparkMax(Constants.Mouth.SPIN_MOUTH_MOTOR, MotorType.kBrushless);
        SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
        spinMotorConfig.idleMode(IdleMode.kCoast);
        spinMotorConfig.smartCurrentLimit(30);
        spinMotorConfig.inverted(true);
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Position Motor
        positionMotor = new SparkMax(Constants.Mouth.POSITION_MOUTH_MOTOR, MotorType.kBrushless);
        motionController = positionMotor.getClosedLoopController();
        SparkMaxConfig positionMotorConfig = new SparkMaxConfig();
        positionMotorConfig.idleMode(IdleMode.kBrake);
        positionMotorConfig.smartCurrentLimit(30);
        positionMotorConfig.inverted(true);
        positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(MouthState.UP, 0.0);
        positions.put(MouthState.DOWN, 0.0);
    }


    @Override
    public boolean matchesState() {
        return getPosition() == positions.get(getCurrentState());
    }


    @Override
    public void init() {
        positionMotor.getEncoder().setPosition(positions.get(MouthState.UP));
    }


    @Override
    public void periodic() {
        switch ((MouthState) getCurrentState()) {
            case UP: 
                goToPosition(positions.get(MouthState.UP));
                break;
            case DOWN:
                goToPosition(positions.get(MouthState.DOWN));
                collect();
                break;
        }
    }

    public double getPosition() {
        System.out.println("current mouth position: " + positionMotor.getEncoder().getPosition());
        return positionMotor.getEncoder().getPosition();
    }

    public void goToPosition(double position) {
        System.out.println("desired mouth position: " + position);
        //motionController.setReference(position, ControlType.kPosition);
    }

    private void collect() {
        //spinMotor.set(0.01);
    }

    public enum MouthState implements SubsystemState {
        UP,
        DOWN
    }
}