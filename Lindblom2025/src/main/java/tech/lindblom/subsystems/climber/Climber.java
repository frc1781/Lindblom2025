package tech.lindblom.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Climber extends BaseClimber {

    private final RelativeEncoder mArmEncoder;

    private ClimberState PreviousState;
    private Rotation2d positionToHold = new Rotation2d();

    private ArmFeedforward armFeedforward;

    public Climber() {
        mArmEncoder = leverMotor.getEncoder();
        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        PreviousState = (ClimberState) this.getCurrentState();
        leverMotor.set(0);
        mArmEncoder.setPosition(0);
    }

    public double getMotorVelocity() {
        return mArmEncoder.getVelocity() / 60;
    }

    @Override
    public void periodic() {
        if (currentMode == EnumCollection.OperatingMode.DISABLED) return;

        Rotation2d mMotorPosition = Rotation2d.fromRadians(mArmEncoder.getPosition());
        double mMotorVelocity = getMotorVelocity();

        Logger.recordOutput("Climber/Velocity", mMotorVelocity);
        Logger.recordOutput("Climber/Position", mMotorPosition);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                leverMotor.set(0.5);
                System.out.println("Climber/Idle");
                break;
            case LIFT:
                leverMotor.set(0.5);
                break;
            case HOLD:
                if (PreviousState != ClimberState.HOLD) {
                    positionToHold = mMotorPosition;
                }

                leverMotor.getClosedLoopController().setReference(positionToHold.getRadians(), SparkBase.ControlType.kVoltage, ClosedLoopSlot.kSlot0, armFeedforward.calculate(positionToHold.getDegrees(), (Math.PI / 2)));
                break;
        }

        PreviousState = (ClimberState)this.getCurrentState();
    }
}
