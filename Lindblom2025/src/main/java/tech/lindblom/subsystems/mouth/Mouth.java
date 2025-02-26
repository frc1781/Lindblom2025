package tech.lindblom.subsystems.mouth;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import tech.lindblom.utils.EEUtil;


public class Mouth extends StateSubsystem {
    private final SparkFlex spinMotor;
    private final SparkMax positionMotor;
    private final SparkClosedLoopController motionController;

    private final HashMap<MouthState, Double> positions = new HashMap<>();

    public Mouth() {
        super("Mouth", MouthState.UP);

        //Spin Motor
        spinMotor = new SparkFlex(Constants.Mouth.SPIN_MOUTH_MOTOR, MotorType.kBrushless);
        SparkFlexConfig spinMotorConfig = new SparkFlexConfig();
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
        positionMotorConfig.closedLoop.pid(0.05, 0, 0);
        positionMotorConfig.encoder.positionConversionFactor((360 * ((double) 1/5) * ((double) 1 /5) * ((double) 1/5)));
        positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positionMotor.getEncoder().setPosition(0.0);

        positions.put(MouthState.UP, 0.0);
        positions.put(MouthState.SPIT, 0.0);
        positions.put(MouthState.EAT, -45.0);

    }


    @Override
    public boolean matchesState() {
        double positionDiff = Math.abs((positions.get(getCurrentState()) - getPosition()));
        double tolerance = 2;
        return positionDiff <= tolerance;
    }


    @Override
    public void init() {
        positionMotor.getEncoder().setPosition(positions.get(MouthState.UP));
    }


    @Override
    public void periodic() {
        switch ((MouthState) getCurrentState()) {
            case UP:
                spinMotor.set(0);
                goToPosition(positions.get(MouthState.UP));
                break;
            case EAT:
                if (spinMotor.getEncoder().getPosition() > 200) {}
                goToPosition(positions.get(MouthState.EAT));
                collect();
                break;
            case SPIT:
                spinMotor.set(-1);
                positionMotor.set(0);
                break;
        }
    }

    public double getPosition() {
        return positionMotor.getEncoder().getPosition();
    }

    public void goToPosition(double position) {
        if (Math.abs(position - getPosition()) > 3) {
            positionMotor.set(EEUtil.clamp(-0.3,0.3, position - getPosition() * 0.01));
        } else {
            positionMotor.set(0);
        }
    }

    private void collect() {
        spinMotor.set(1);
    }

    public enum MouthState implements SubsystemState {
        UP,
        EAT,
        SPIT,
        OPEN,
        CLOSE
    }
}