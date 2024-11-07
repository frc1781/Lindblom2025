package tech.lindblom.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.routines.TestRoutine;
import tech.lindblom.subsystems.drive.Drive;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.EnumCollection;

import java.util.ArrayList;
import java.util.Optional;

public class RobotController {
    Drive driveSystem;
    Vision visionSystem;
    Auto autoSystem;

    private ArrayList<StateSubsystem> stateSubsystems;
    private ArrayList<Subsystem> subsystems;

    public RobotController() {
        driveSystem = new Drive();
        autoSystem = new Auto(
                new TestRoutine()
        );

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
                    setInitialRobotPose();
                break;
            case TELEOP:
                    setInitialRobotPose();
                break;
            case TEST:
                break;
            case SIMULATION:
                break;
            default:
                System.out.println("WHAT IS HAPPENING");
                break;
        }

        for (Subsystem subsystem : subsystems) {
            subsystem.setOperatingMode(mode);
        }
    }

    public void run(EnumCollection.OperatingMode mode) {
        switch (mode) {
            case DISABLED:
                    visionUpdates();
                break;
            case AUTONOMOUS:
                    visionUpdates();
                break;
            case TELEOP:
                    visionUpdates();
                break;
            case TEST:
                break;
            case SIMULATION:
                break;
            default:
                System.out.println("WHAT IS HAPPENING");
                break;
        }

        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void visionUpdates() {
        Optional<Pose2d> visionEstimateOptional = visionSystem.getFrontCameraPose();
        if (visionEstimateOptional.isPresent()) {
            Pose2d visionEstimate = visionEstimateOptional.get();
            driveSystem.updatePoseUsingVisionEstimate(
                    visionEstimate,
                    Timer.getFPGATimestamp(),
                    visionSystem.getEstimationStdDevs(
                            visionEstimate,
                            visionSystem.getFrontCameraPipelineResult()
                    )
            );
        }
    }

    public void setInitialRobotPose(EnumCollection.OperatingMode mode) {
        if (mode == EnumCollection.OperatingMode.AUTONOMOUS) {
            try {
                Pose2d poseFromPath = autoSystem.getStartPosition();
                driveSystem.setInitialPose(poseFromPath);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (mode == EnumCollection.OperatingMode.TELEOP) {

        }
    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
