package tech.lindblom.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.playingwithfusion.TimeOfFlight;
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
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection;

import static tech.lindblom.utils.EnumCollection.OperatingMode.*;

public class DriveController extends StateSubsystem {
    private final Drive driveSubsystem;
    private final RobotController robotController;
    private PathPlannerPath followingPath;
    private PathPlannerTrajectory followingTrajectory;
    private Pose2d targetPose;
    private HolonomicDriveController trajectoryController;

    private final TimeOfFlight armTOF;

    private final PIDController XController = new PIDController(3, 0, 0);
    private final PIDController YController = new PIDController(3, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(4, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));

    private final PIDController centeringYawController = new PIDController(0.025, 0, 0);
    private final PIDController distanceController = new PIDController(1.6  , 0, 0);

    private final ChassisSpeeds zeroSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
    private RobotConfig robotConfig;
    private boolean isFieldOriented = true;

    public DriveController(RobotController controller) {
        super("DriveController", DriverStates.IDLE);
        driveSubsystem = new Drive();
        robotController = controller;
        armTOF = new TimeOfFlight(Constants.Drive.ARM_TOF_ID);

        rotController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void init() {
        centeringYawController.reset();
        distanceController.reset();

        switch (currentOperatingMode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
                driveSubsystem.resetNavX();
                driveSubsystem.zeroRotation();
                setInitialRobotPose(currentOperatingMode);
                break;
            case TELEOP:
                setInitialRobotPose(currentOperatingMode);
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
        if (currentOperatingMode == DISABLED) return;
        driveSubsystem.periodic();

        switch ((DriverStates) getCurrentState()) {
            case IDLE:
                driveSubsystem.drive(zeroSpeed);
                break;
            case CENTERING:
                centerOnReef();
                break;
            case DRIVER:
                //RobotController is inputing speeds from driver input
                break;
            case PATH:
                boolean hasRobotReachedTargetPose = hasReachedTargetPose();
                Logger.recordOutput(name + "/hasRobotReachedTargetPose", hasRobotReachedTargetPose);

                if (hasRobotReachedTargetPose || followingPath == null) {
                    driveSubsystem.drive(zeroSpeed);
                }

                Logger.recordOutput(name + "/isFollowingPath", followingPath != null && !hasRobotReachedTargetPose);
                if (followingPath != null && !hasRobotReachedTargetPose) {
                    followPath();
                }
                break;
            case FIND_POLE:
                findReefPole();
                break;
        }
    }

    public void resetNavX() {
        driveSubsystem.zeroRotation();
    }

    public void driveUsingVelocities(double xVelocity, double yVelocity, double rotSpeed) {
        if (getCurrentState() != DriverStates.DRIVER) return;

        Logger.recordOutput(this.name + "/xVelocity", xVelocity);
        Logger.recordOutput(this.name + "/yVelocity", yVelocity);
        Logger.recordOutput(this.name + "/rotSpeed", rotSpeed);


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

            if (currentOperatingMode == AUTONOMOUS && cameraDistance > 1.5) {
                return inputSpeeds;
            } else if (currentOperatingMode == AUTONOMOUS){
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

    public void centerOnReef() {
        if (robotController.getCenteringSide() == null) return;
        ChassisSpeeds centeringSpeeds = zeroSpeed;
        centeringSpeeds = getCenteringChassisSpeeds(centeringSpeeds);
        driveSubsystem.drive(centeringSpeeds);
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

    public void findReefPole() {
        if (robotController.getCenteringSide() == null) return;


    }

    public boolean hasFoundReefPole() {
        return armTOF.getRange() < Constants.Drive.ARM_TOF_DISTANCE;
    }

    public void setAutoPath(PathPlannerPath path) {
        followingPath = path;
        followingTrajectory = null;
        if (path == null) return;

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
        PathPlannerTrajectoryState pathplannerState = followingTrajectory.sample(robotController.autoTimer.get());
        Pose2d targetPose = new Pose2d(pathplannerState.pose.getTranslation(), pathplannerState.heading);
        Rotation2d targetOrientation = EEUtil.normalizeAngle(pathplannerState.pose.getRotation());
        Logger.recordOutput(name + "/TrajectoryPose", targetPose);

        ChassisSpeeds desiredChassisSpeeds = trajectoryController.calculate(
            driveSubsystem.getRobotPose(),
            targetPose,
            pathplannerState.linearVelocity,
                targetOrientation
        );

        driveSubsystem.drive(desiredChassisSpeeds);
    }

    public boolean hasReachedTargetPose() {
        if (targetPose == null) return true;

        Pose2d currentPose = driveSubsystem.getRobotPose();
        boolean reachedTargetTranslation = currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1;
        boolean reachedTargetHeading = true;

        return (reachedTargetHeading && reachedTargetTranslation) || robotController.shouldBeCentering();
    }

    public void updatePoseUsingVisionEstimate(Pose2d estimatedPose, double time, Matrix<N3, N1> stdValue) {
        Logger.recordOutput(this.name + "/VisionEstimatedPose", estimatedPose);
        driveSubsystem.updatePoseUsingVisionEstimate(new Pose2d(estimatedPose.getTranslation(), driveSubsystem.getRobotRotation()), time, stdValue);
    }

    public void setInitialRobotPose(EnumCollection.OperatingMode mode) {
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

    @Override
    public boolean matchesState() {
        switch ((DriverStates) getCurrentState()) {
            case CENTERING:
                return hasFinishedCentering();
            case PATH:
                return hasReachedTargetPose();
            case FIND_POLE:
                return hasFoundReefPole();
            case IDLE:
                return false;
            case DRIVER:
                return false;
            default:
                return false;
        }
    }

    public enum DriverStates implements SubsystemState {
        IDLE,
        DRIVER,
        CENTERING,
        PATH,
        FIND_POLE
    }
}
