package tech.lindblom.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PWM;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;
import tech.lindblom.utils.EEUtil;

public class Climber extends BaseClimber {

    private final RelativeEncoder armEncoder;
    private double requestedPosition;
    private boolean climbing;
    private double climberDutyCycle;
    private PIDController climberPID;

    public Climber() {
        armEncoder = leverMotor.getEncoder();
        climbing = false;
        requestedPosition = armEncoder.getPosition();
        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);
        climberDutyCycle = 0.0;
        climberPID = new PIDController(0.1, 0, 0);
        climberPID.reset();
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        leverMotor.set(0);
        armEncoder.setPosition(0);
        requestedPosition = armEncoder.getPosition();
        climberDutyCycle = 0.0;
    }

    public double getMotorVelocity() {
        return armEncoder.getVelocity() / 60;
    }

    public void disableClimb() {
        climbing = false;
    }

    @Override
    public void periodic() {
        if (currentOperatingMode != EnumCollection.OperatingMode.TELEOP) {
            return;
        }

        Rotation2d mMotorPosition = Rotation2d.fromRadians(armEncoder.getPosition());
        double mMotorVelocity = getMotorVelocity();

        Logger.recordOutput("Climber/Velocity", mMotorVelocity);
        Logger.recordOutput("Climber/Position", mMotorPosition);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                requestedPosition = armEncoder.getPosition();
                break;
            case DOWN:
                requestedPosition += 1;
                if (!climbing) {
                    climberPID.reset();
                }
                climbing = true;
                break;
            case UP:
                requestedPosition -= 1;
                if (!climbing) {
                    climberPID.reset();
                }
                climbing = true;
                break;
        }


        if (climbing) {
            climberDutyCycle = climberPID.calculate(armEncoder.getPosition(), requestedPosition);
        }
        else {
            climberDutyCycle = 0.0;
        }

        climberDutyCycle = EEUtil.clamp(-0.1, 0.1, climberDutyCycle);
        leverMotor.set(climberDutyCycle);
        Logger.recordOutput("Climber/dutyCycle", climberDutyCycle);
    }
}
