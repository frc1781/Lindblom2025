package tech.lindblom.control;

import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.drive.Drive;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.EnumCollection;

import java.util.ArrayList;

public class ControlSystem {
    Drive driveSystem;
    Vision visionSystem;
    Auto autoSystem;

    private ArrayList<StateSubsystem> stateSubsystems;
    private ArrayList<Subsystem> subsystems;

    public ControlSystem() {
        driveSystem = new Drive();
        autoSystem = new Auto();

        visionSystem = new Vision();

        stateSubsystems = new ArrayList<>();
        stateSubsystems.add(driveSystem);

        subsystems = new ArrayList<>();
        subsystems.add(visionSystem);
        subsystems.add(autoSystem);
    }

    public enum Action {
        WAIT
    }

    public void init(EnumCollection.OperatingMode mode) {
        switch (mode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                break;
            case TELEOP:
                break;
            case TEST:
                break;
            case SIMULATION:
                break;
            default:
                System.out.println("WHAT IS HAPPENING");
                break;
        }
    }

    public void run(EnumCollection.OperatingMode mode) {
        switch (mode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                break;
            case TELEOP:
                break;
            case TEST:
                break;
            case SIMULATION:
                break;
            default:
                System.out.println("WHAT IS HAPPENING");
                break;
        }
    }
}
