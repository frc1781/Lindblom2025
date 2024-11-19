package tech.lindblom.subsystems.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.EEGeometeryUtil;
import tech.lindblom.utils.EnumCollection;

import java.util.Optional;

import static tech.lindblom.utils.EnumCollection.OperatingMode.*;

public class DriveController extends Subsystem {
    private final Drive driveSubsystem;
    private final RobotController robotController;
    private PathPlannerPath followingPath;
    private PathPlannerTrajectory followingTrajectory;
    private Pose2d targetPose;
    private HolonomicDriveController trajectoryController;

    private final PIDController XController = new PIDController(3, 0, 0);
    private final PIDController YController = new PIDController(3, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(4, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));

    private final ChassisSpeeds zeroSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DriveController(RobotController controller) {
        super("DriveController");
        driveSubsystem = new Drive();
        robotController = controller;
    }

    @Override
    public void init() {
        switch (currentMode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                driveSubsystem.navX.reset();
                driveSubsystem.navX.zeroYaw();
                setInitialRobotPose(currentMode);
                break;
            case TELEOP:
                setInitialRobotPose(currentMode);
                break;
        }
    }

    @Override
    public void periodic() {
        driveSubsystem.periodic();

        switch (currentMode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                boolean hasRobotReachedTargetPose = hasRobotReachedTargetPose();
                Logger.recordOutput(name + "/hasRobotReachedTargetPose", hasRobotReachedTargetPose);

                if (hasRobotReachedTargetPose() && trajectoryController != null) {
                    setAutoPath(null);
                    driveSubsystem.drive(zeroSpeed);
                }

                if (followingPath != null && !hasRobotReachedTargetPose) {
                    Logger.recordOutput(name + "/isFollowingPath", true);
                    followPath();
                } else {
                    Logger.recordOutput(name + "/isFollowingPath", false);
                }
                break;
        }
    }

    public void resetNavX() {
        driveSubsystem.zeroNavX();
    }


    public void driveUsingVelocities(double xVelocity, double yVelocity, double rotSpeed) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                xVelocity,
                yVelocity,
                rotSpeed), driveSubsystem.getRobotRotation());
        driveSubsystem.drive(speeds);
    }

    public void setAutoPath(PathPlannerPath path) {
        followingPath = path;
        if (path == null) return;

        followingTrajectory = path.getTrajectory(new ChassisSpeeds(), driveSubsystem.getRobotRotation());

        targetPose = followingTrajectory.getEndState().getTargetHolonomicPose();
        Logger.recordOutput(name + "/TargetPose", targetPose);

        PIDController xTrajectoryController;
        PIDController yTrajectoryController;
        ProfiledPIDController rotTrajectoryController;
        xTrajectoryController = new PIDController(10, 0.0, 0.001);
        yTrajectoryController = new PIDController(10, 0.0, 0.001);
        rotTrajectoryController = new ProfiledPIDController(5.5, 0.01, 0.01,
                new TrapezoidProfile.Constraints(6.28, 12.14));
        rotTrajectoryController.enableContinuousInput(0, 2 * Math.PI);
        trajectoryController = new HolonomicDriveController(xTrajectoryController, yTrajectoryController,
                rotTrajectoryController);


        XController.reset();
        YController.reset();
        rotController.reset(driveSubsystem.getRobotPose().getRotation().getRadians());
    }

    public void followPath() {
        if (hasRobotReachedTargetPose() || followingPath == null) return;

        PathPlannerTrajectory.State pathplannerState = followingTrajectory.sample(robotController.autoTimer.get());
        Pose2d targetPose = new Pose2d(pathplannerState.positionMeters, pathplannerState.heading);
        Rotation2d targetOrientation = EEGeometeryUtil.normalizeAngle(pathplannerState.getTargetHolonomicPose().getRotation());
        Logger.recordOutput(name + "/TrajectoryPose", targetPose);

        ChassisSpeeds desiredChassisSpeeds = trajectoryController.calculate(
            driveSubsystem.getRobotPose(),
            targetPose,
            pathplannerState.velocityMps,
                targetOrientation
        );

        driveSubsystem.drive(desiredChassisSpeeds);
    }

    public boolean hasRobotReachedTargetPose() {
        if (targetPose == null) return true;

        Pose2d currentPose = driveSubsystem.getRobotPose();
        boolean reachedTargetTranslation = currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1;
        boolean reachedTargetHeading = true;

        return reachedTargetHeading && reachedTargetTranslation;
    }

    public void updatePoseUsingVisionEstimate(Pose2d estimatedPose, double time, Matrix<N3, N1> stdValue) {
        driveSubsystem.updatePoseUsingVisionEstimate(new Pose2d(estimatedPose.getTranslation(), driveSubsystem.getRobotRotation()), time, stdValue);
    }

    public void setInitialRobotPose(EnumCollection.OperatingMode mode) {
        Optional<Pose2d> visionPoseOptional = robotController.visionSystem.getFrontCameraPose();
        PhotonPipelineResult pipelineResult = robotController.visionSystem.getFrontCameraPipelineResult();

        if (visionPoseOptional.isPresent() && pipelineResult != null) {
            Pose2d visionPose = visionPoseOptional.get();

            if (pipelineResult.targets.size() > 1) {
                driveSubsystem.setInitialPose(new Pose2d(visionPose.getTranslation(), driveSubsystem.getNavXRotation()));
                return;
            }
        }

        if (mode == AUTONOMOUS) {
            try {
                Pose2d poseFromPath = robotController.autoSystem.getStartPosition();
                driveSubsystem.setInitialPose(poseFromPath);
            } catch (Exception e) {
                e.printStackTrace();
                driveSubsystem.setInitialPose(new Pose2d());
            }
        } else if (mode == TELEOP) {
            double startingDegRotation = RobotController.isRed() ? 180 : 0;
            driveSubsystem.setInitialPose(new Pose2d(new Translation2d(), new Rotation2d(startingDegRotation)));
        }
    }
}
