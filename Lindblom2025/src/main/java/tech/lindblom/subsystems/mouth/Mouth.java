package tech.lindblom.subsystems.mouth;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.util.Units;
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
        spinMotorConfig.inverted(false);
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Position Motor
        positionMotor = new SparkMax(Constants.Mouth.POSITION_MOUTH_MOTOR, MotorType.kBrushless);
        motionController = positionMotor.getClosedLoopController();
        SparkMaxConfig positionMotorConfig = new SparkMaxConfig();
        positionMotorConfig.idleMode(IdleMode.kBrake);
        positionMotorConfig.smartCurrentLimit(30);
        positionMotorConfig.inverted(false);
        positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(MouthState.UP, 0.0);
        positions.put(MouthState.DOWN, 0.0);
    }


    @Override
    public boolean matchesState() {
        System.out.println("matches state: " + matchesState());
        double positionDiff = Math.abs((positions.get(getCurrentState()) - getPosition()));
        double tolerance = 2;
        return positionDiff <= tolerance;
        
    }


    @Override
    public void init() {
        System.out.println("starting position: " + positionMotor.getEncoder().getPosition());
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
        motionController.setReference(position, ControlType.kPosition);
    }

    private void collect() {
        System.out.println("current speed: " + spinMotor.get());
        spinMotor.set(0.01);
    }

    public enum MouthState implements SubsystemState {
        UP,
        DOWN
    }
}