package tech.lindblom.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import tech.lindblom.utils.EnumCollection;

public class ClimberSim extends BaseClimber {

    private final SparkMaxSim armMotorSim;
    private final SparkRelativeEncoderSim armEncoder;
    private final SingleJointedArmSim armSim;


    private ClimberState PreviousState;
    private Rotation2d positionToHold = new Rotation2d();

    public ClimberSim() {
        DCMotor NEO = DCMotor.getNEO(1);
        NEO.withReduction(125);
        armMotorSim = new SparkMaxSim(leverMotor, NEO);
        armEncoder = armMotorSim.getRelativeEncoderSim();
        armSim = new SingleJointedArmSim(NEO,
                1,
                5.6,
                0.3,
                0.5,
                1,
                true,
                0,
                0);
    }

    @Override
    public boolean matchesState() {
        return getCurrentState() == ClimberState.IDLE;
    }

    @Override
    public void init() {
        PreviousState = (ClimberState) this.getCurrentState();
        leverMotor.set(0);
        armEncoder.setPosition(0);
    }

    public double getMotorVelocity() {
        return armEncoder.getVelocity() / 60;
    }

    @Override
    public void periodic() {
        if (currentMode == EnumCollection.OperatingMode.DISABLED) return;

        Rotation2d mMotorPosition = Rotation2d.fromRadians(armEncoder.getPosition());
        double mMotorVelocity = getMotorVelocity();

        Logger.recordOutput("Climber/Velocity", mMotorVelocity);
        Logger.recordOutput("Climber/Position", mMotorPosition);

        switch((ClimberState)getCurrentState()) {
            case IDLE:
                //armMotorSim.se
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
