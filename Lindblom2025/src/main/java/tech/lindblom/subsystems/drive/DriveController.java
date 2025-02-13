package tech.lindblom.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
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
import tech.lindblom.control.DriverInput;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEGeometeryUtil;
import tech.lindblom.utils.EnumCollection;

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

    private final PIDController centeringYawController = new PIDController(0.025, 0, 0);
    private final PIDController distanceController = new PIDController(1.6  , 0, 0);

    private boolean aprilTagControl = false;

    private final ChassisSpeeds zeroSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
    private RobotConfig robotConfig;
    private boolean isFieldOriented = true;

    public DriveController(RobotController controller) {
        super("DriveController");
        driveSubsystem = new Drive();
        robotController = controller;

        rotController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void init() {
        centeringYawController.reset();
        distanceController.reset();

        switch (currentMode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                driveSubsystem.resetNavX();
                driveSubsystem.zeroRotation();
                setInitialRobotPose(currentMode);
                break;
            case TELEOP:
                setInitialRobotPose(currentMode);
                break;
        }

        try{
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        driveSubsystem.periodic();

        switch (currentMode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                boolean hasRobotReachedTargetPose = (hasRobotReachedTargetPose() && robotController.getCenteringSide() == null) || hasFinishedCentering();
                Logger.recordOutput(name + "/hasRobotReachedTargetPose", hasRobotReachedTargetPose);

                if (hasRobotReachedTargetPose || followingPath == null) {
                    driveSubsystem.drive(zeroSpeed);
                }

                if (followingPath != null && !hasRobotReachedTargetPose) {
                    Logger.recordOutput(name + "/isFollowingPath", true);
                    followPath();
                } else {
                    Logger.recordOutput(name + "/isFollowingPath", false);
                }
                break;
            case TELEOP:
                break;
        }
    }

    public void resetNavX() {
        driveSubsystem.zeroRotation();
    }


    public void driveUsingVelocities(double xVelocity, double yVelocity, double rotSpeed) {
        ChassisSpeeds speeds = isFieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                        xVelocity,
                        yVelocity,
                        rotSpeed),
                driveSubsystem.getRotation())
                : new ChassisSpeeds(xVelocity, yVelocity, rotSpeed);

        speeds = getCenteringChassisSpeeds(speeds);

        driveSubsystem.drive(speeds);
    }

    public ChassisSpeeds getCenteringChassisSpeeds(ChassisSpeeds inputSpeeds) {
        if (robotController.getCenteringSide() == null) return inputSpeeds;

        int apriltagId = 0;
        double cameraOffset = 0.0;
        double cameraDistance = 0.0;

        if (robotController.getCenteringSide() == DriverInput.ReefCenteringSide.LEFT) {
            apriltagId = robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_LEFT);
            if (apriltagId != -1) {
                cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_LEFT, apriltagId);
                cameraDistance = robotController.visionSystem.getCameraDistanceX(Vision.Camera.FRONT_LEFT, apriltagId);
            }

            if (currentMode == AUTONOMOUS && cameraDistance > 1.5) {
                return inputSpeeds;
            } else if (currentMode == AUTONOMOUS){
                aprilTagControl = true;
                inputSpeeds = zeroSpeed;
            }

            if (cameraOffset != 1781 &&  cameraDistance != 1781) {
                if (!(Math.abs(7 - cameraOffset) < 0.02)) {
                    inputSpeeds.vyMetersPerSecond = centeringYawController.calculate(cameraOffset, 7);
                }

                if (!(Math.abs(Constants.Drive.TARGET_CORAL_DISTANCE - cameraDistance) < 0.05))
                inputSpeeds.vxMetersPerSecond = -distanceController.calculate(cameraDistance, Constants.Drive.TARGET_CORAL_DISTANCE); // not sure why this needs a negative sign
            }
        }

        Logger.recordOutput(this.name + "/driveUsingVelocities", inputSpeeds);
        Logger.recordOutput(this.name + "/cameraOffset", cameraOffset);
        Logger.recordOutput(this.name + "/cameraDistance", cameraDistance);
        Logger.recordOutput(this.name + "/apriltagId", apriltagId);

        return inputSpeeds;
    }

    public boolean hasFinishedCentering() {
        if (robotController.getCenteringSide() == null) return false;
        int apriltagId = 0;
        double cameraOffset = 0.0;
        double cameraDistance = 0.0;

        if (robotController.getCenteringSide() == DriverInput.ReefCenteringSide.LEFT) {
            apriltagId = robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_LEFT);
            if (apriltagId != -1) {
                cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_LEFT, apriltagId);
                cameraDistance = robotController.visionSystem.getCameraDistanceX(Vision.Camera.FRONT_LEFT, apriltagId);
                Logger.recordOutput(this.name + "/HasFinishedCentering", Math.abs(8.1 - cameraOffset) < 0.3 && Math.abs(Constants.Drive.TARGET_CORAL_DISTANCE - cameraDistance) < 0.05);
                return Math.abs(7 - cameraOffset) < 0.3 && Math.abs(Constants.Drive.TARGET_CORAL_DISTANCE - cameraDistance) < 0.05;
            }
        }

        return false;
    }

    public void setAutoPath(PathPlannerPath path) {
        followingPath = path;
        followingTrajectory = null;
        if (path == null) return;

        aprilTagControl = false;
        followingTrajectory = path.generateTrajectory(new ChassisSpeeds(), driveSubsystem.getRobotRotation(), robotConfig);

        targetPose = followingTrajectory.getEndState().pose;
        Logger.recordOutput(name + "/TargetPose", targetPose);

        PIDController xTrajectoryController;
        PIDController yTrajectoryController;
        ProfiledPIDController rotTrajectoryController;
        xTrajectoryController = new PIDController(1, 0.0, 0.001);
        yTrajectoryController = new PIDController(1, 0.0, 0.001);
        rotTrajectoryController = new ProfiledPIDController(5.5, 0.01, 0.01,
                new TrapezoidProfile.Constraints(Math.PI * 2, 12.14));
        rotTrajectoryController.enableContinuousInput(0, 2 * Math.PI);
        trajectoryController = new HolonomicDriveController(xTrajectoryController, yTrajectoryController,
                rotTrajectoryController);


        XController.reset();
        YController.reset();
        rotController.reset(driveSubsystem.getRobotPose().getRotation().getRadians());
    }


    public void followPath() {
        if (followingPath == null) return;

        PathPlannerTrajectoryState pathplannerState = followingTrajectory.sample(robotController.autoTimer.get());
        Pose2d targetPose = new Pose2d(pathplannerState.pose.getTranslation(), pathplannerState.heading);
        Rotation2d targetOrientation = EEGeometeryUtil.normalizeAngle(pathplannerState.pose.getRotation());
        Logger.recordOutput(name + "/TrajectoryPose", targetPose);
        Logger.recordOutput(name + "/aprilTagControl", aprilTagControl);

        ChassisSpeeds desiredChassisSpeeds = trajectoryController.calculate(
            driveSubsystem.getRobotPose(),
            targetPose,
            pathplannerState.linearVelocity,
                targetOrientation
        );

        desiredChassisSpeeds = getCenteringChassisSpeeds(desiredChassisSpeeds);

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
        Logger.recordOutput(this.name + "/VisionEstimatedPose", estimatedPose);
        driveSubsystem.updatePoseUsingVisionEstimate(new Pose2d(estimatedPose.getTranslation(), driveSubsystem.getRobotRotation()), time, stdValue);
    }

    public void setInitialRobotPose(EnumCollection.OperatingMode mode) {
/*        Optional<Pose2d> visionPoseOptional = robotController.visionSystem.getFrontCameraPose();
        PhotonPipelineResult pipelineResult = robotController.visionSystem.getFrontCameraPipelineResult();

        if (visionPoseOptional.isPresent() && pipelineResult != null) {
            Pose2d visionPose = visionPoseOptional.get();

            if (pipelineResult.targets.size() > 1) {
                driveSubsystem.setInitialPose(new Pose2d(visionPose.getTranslation(), driveSubsystem.getNavXRotation()));
                return;
            }
        }*/

        if (mode == AUTONOMOUS) {
            try {
                Pose2d poseFromPath = robotController.autoSystem.getStartPosition();
                driveSubsystem.setInitialPose(poseFromPath);
            } catch (Exception e) {
                e.printStackTrace();
                driveSubsystem.setInitialPose(new Pose2d());
            }
        } else if (mode == TELEOP) {
            double startingDegRotation = RobotController.isRed() ? 0 : 180;
            driveSubsystem.setInitialPose(new Pose2d(new Translation2d(), new Rotation2d(startingDegRotation)));
        }
    }
}
