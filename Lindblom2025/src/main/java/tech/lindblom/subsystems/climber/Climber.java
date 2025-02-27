package tech.lindblom.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PWM;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Climber extends BaseClimber {

    private final RelativeEncoder armEncoder;

    public Climber() {
        latchServo = new PWM(Constants.Climber.SERVO_PWM_PORT);
        armEncoder = leverMotor.getEncoder();
        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        leverMotor.set(0);
        armEncoder.setPosition(0);
    }

    public double getMotorVelocity() {
        return armEncoder.getVelocity() / 60;
    }

    @Override
    public void periodic() {
        if (currentOperatingMode == EnumCollection.OperatingMode.DISABLED) return;

        Rotation2d mMotorPosition = Rotation2d.fromRadians(armEncoder.getPosition());
        double mMotorVelocity = getMotorVelocity();

        Logger.recordOutput("Climber/Velocity", mMotorVelocity);
        Logger.recordOutput("Climber/Position", mMotorPosition);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                latchServo.setSpeed(0);
                leverMotor.set(0);
                break;
            case DOWN:
                latchServo.setSpeed(0);
                leverMotor.set(0.5);
                break;
            case UP:
                latchServo.setSpeed(0);
                leverMotor.set(-0.5);
                break;
            case RELEASE_LATCH:
                latchServo.setPosition(0.5);
                break;
        }
    }
}
