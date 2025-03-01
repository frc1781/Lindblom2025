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
    private TimeOfFlight leftTOF;
    private TimeOfFlight rightTOF;
    private final TimeOfFlight armTOF;

    private final PIDController XController = new PIDController(3, 0, 0);
    private final PIDController YController = new PIDController(3, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(4, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));

    private final PIDController centeringYawController = new PIDController(0.015, 0, 0);
    private final PIDController distanceController = new PIDController(1, 0, 0);
    private final ProfiledPIDController parallelController = new ProfiledPIDController(0.1, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));

    private RobotConfig robotConfig;
    private boolean isFieldOriented = true;
    private boolean reachedDesiredDistance = false;
    private boolean detectedPole = false;

    public DriveController(RobotController controller) {
        super("DriveController", DriverStates.IDLE);
        driveSubsystem = new Drive();
        robotController = controller;
        armTOF = new TimeOfFlight(Constants.Drive.ARM_TOF_ID);
        armTOF.setRangingMode(TimeOfFlight.RangingMode.Short, 20);
        armTOF.setRangeOfInterest(6, 1, 10, 16);
        leftTOF = new TimeOfFlight(Constants.Drive.LEFT_FRONT_TOF_ID);
        leftTOF.setRangingMode(TimeOfFlight.RangingMode.Short, 20);
        rightTOF = new TimeOfFlight(Constants.Drive.RIGHT_FRONT_TOF_ID);
        rightTOF.setRangingMode(TimeOfFlight.RangingMode.Short, 20);

        rotController.enableContinuousInput(0, Math.PI * 2);
        parallelController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void init() {
        centeringYawController.reset();
        distanceController.reset();
        parallelController.reset(driveSubsystem.getRobotPose().getRotation().getRadians());

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
        Logger.recordOutput(this.name + "/armTOF", armTOF.getRange());
        Logger.recordOutput(this.name + "/armTOFVaild", armTOF.isRangeValid());
        Logger.recordOutput(this.name + "/leftTOF", leftTOF.getRange());
        Logger.recordOutput(this.name + "/leftTOFVaild", leftTOF.isRangeValid());
        Logger.recordOutput(this.name + "/rightTOF", rightTOF.getRange());
        Logger.recordOutput(this.name + "/rightTOFVaild", rightTOF.isRangeValid());

        int apriltagId = robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_RIGHT);
        if (apriltagId != -1) {
            double cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_RIGHT, apriltagId);
            double cameraDistance = robotController.visionSystem.getCameraDistanceX(Vision.Camera.FRONT_RIGHT, apriltagId);
            Logger.recordOutput(this.name + "/apriltagId", apriltagId);
            Logger.recordOutput(this.name + "/cameraOffset", cameraOffset);
            Logger.recordOutput(this.name + "/cameraDistance", cameraDistance);
        }

        if (currentOperatingMode == DISABLED) return;

        switch ((DriverStates) getCurrentState()) {
            case IDLE:
                driveSubsystem.drive(zeroSpeed());
                break;
            case CENTERING_LEFT,CENTERING_RIGHT:
                centerOnReef();
                break;
            case DRIVER:
                //RobotController is inputing speeds from driver input
                break;
            case PATH:
                boolean hasRobotReachedTargetPose = hasReachedTargetPose();
                Logger.recordOutput(name + "/hasRobotReachedTargetPose", hasRobotReachedTargetPose);

                if (hasRobotReachedTargetPose || followingPath == null) {
                    driveSubsystem.drive(zeroSpeed());
                }

                Logger.recordOutput(name + "/isFollowingPath", followingPath != null && !hasRobotReachedTargetPose);
                if (followingPath != null && !hasRobotReachedTargetPose) {
                    followPath();
                }
                break;
        }

        driveSubsystem.periodic();
    }

    private ChassisSpeeds zeroSpeed() {
        return new ChassisSpeeds(0.0, 0.0, 0.0);
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

        driveSubsystem.drive(speeds);
    }

    private boolean areValidCameraReading(double cameraOffset) {
       return 
        cameraOffset != Constants.Vision.ERROR_CONSTANT &&
        cameraOffset != 0.0; //0.0 indicates it is not estimating distance
    }

    public ChassisSpeeds getCenteringChassisSpeeds(ChassisSpeeds inputSpeeds) {
        if (robotController.getCenteringSide() == null) 
            return inputSpeeds;

        int apriltagId = 0;
        double cameraOffset = 0.0;
        double cameraDistance = 0.0;
        double targetOffset;
        
        if (robotController.getCenteringSide() == DriverInput.ReefCenteringSide.LEFT) {
            targetOffset = Constants.Drive.TARGET_CORAL_OFFSET_LEFT;
            apriltagId = robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_RIGHT);
            if (apriltagId != -1) {
                cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_RIGHT, apriltagId);
                cameraDistance = robotController.visionSystem.getCameraDistanceX(Vision.Camera.FRONT_RIGHT, apriltagId);
            }
        } else {
            targetOffset = Constants.Drive.TARGET_CORAL_OFFSET_RIGHT;
            apriltagId = robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_LEFT);
            if (apriltagId != -1) {
                cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_LEFT, apriltagId);
                cameraDistance = robotController.visionSystem.getCameraDistanceX(Vision.Camera.FRONT_LEFT, apriltagId);
            }
        }

        if (currentOperatingMode == AUTONOMOUS) {
            return cameraDistance < 1 ? inputSpeeds : zeroSpeed();
        }

        if (apriltagId == -1) {
            inputSpeeds.vyMetersPerSecond = EEUtil.clamp(-0.1, 0.1, getCurrentState() == DriverStates.CENTERING_RIGHT ? -0.1 : 0.1);
        }
        if (Math.abs(targetOffset - cameraOffset) > Constants.Drive.OFFSET_TOLERANCE && areValidCameraReading(cameraOffset)) {
            inputSpeeds.vyMetersPerSecond = centeringYawController.calculate(cameraOffset, targetOffset);
        } else {
            inputSpeeds.vyMetersPerSecond = 0;
        }

        double leftTOFdistance = leftTOF.getRange();
        double rightTOFdistance = rightTOF.getRange();
        if (leftTOF.isRangeValid() && rightTOF.isRangeValid()) {
            if (Math.abs((leftTOFdistance + rightTOFdistance) / 2.0 - Constants.Drive.TARGET_TOF_PARALLEL_DISTANCE) >= 50) {
                inputSpeeds.vxMetersPerSecond = EEUtil.clamp(-0.5, 0.5, 0.005 * ((leftTOFdistance + rightTOFdistance) / 2.0 - Constants.Drive.TARGET_TOF_PARALLEL_DISTANCE));
            } else {
                inputSpeeds.vxMetersPerSecond = 0;
            }

            if (Math.abs(rightTOFdistance - leftTOFdistance) >= 30 && Math.abs(rightTOFdistance - leftTOFdistance) < 200) {
                inputSpeeds.omegaRadiansPerSecond = EEUtil.clamp(-0.5, 0.5, 0.005 * (rightTOFdistance - leftTOFdistance));
            } else {
                inputSpeeds.omegaRadiansPerSecond = 0;
            }

            if (inputSpeeds.omegaRadiansPerSecond == 0 && inputSpeeds.vxMetersPerSecond == 0) {
                reachedDesiredDistance = true;
            }
        }

        Logger.recordOutput(this.name + "/parallelDistance", rightTOFdistance - leftTOFdistance);
        Logger.recordOutput(this.name + "/inCenteredPosition", reachedDesiredDistance);
        Logger.recordOutput(this.name + "/driveUsingVelocities", inputSpeeds);
        Logger.recordOutput(this.name + "/cameraOffset", cameraOffset);
        Logger.recordOutput(this.name + "/cameraDistance", cameraDistance);
        Logger.recordOutput(this.name + "/apriltagId", apriltagId);
        Logger.recordOutput(this.name + "/leftTOFDistance", leftTOF.getRange());
        Logger.recordOutput(this.name + "/rightTOFDistance", rightTOF.getRange());
        Logger.recordOutput(this.name + "/poleTOFdDistance", armTOF.getRange());
        Logger.recordOutput(this.name + "/hasFoundReefPole", hasFoundReefPole());

        if (reachedDesiredDistance && hasFoundReefPole()) {
            return zeroSpeed();
        }

        return inputSpeeds;
    }

    @Override
    public void stateTransition(SubsystemState previousState, SubsystemState newState) {
        if (newState == DriverStates.CENTERING_LEFT || newState == DriverStates.CENTERING_RIGHT) {
            detectedPole = false;
            reachedDesiredDistance = false;
        }
    }

    public void centerOnReef() {
        if (robotController.getCenteringSide() == null) {
            return;
        }

        ChassisSpeeds centeringSpeeds = getCenteringChassisSpeeds(zeroSpeed());
        driveSubsystem.drive(centeringSpeeds);
    }

    public boolean hasFinishedCentering() {
        if (robotController.getCenteringSide() == null)  {
            return false;
        }

        return reachedDesiredDistance && hasFoundReefPole();
    }

    public boolean hasFoundReefPole() {
        boolean hasFoundReefPole = armTOF.getRange() < Constants.Drive.ARM_TOF_DISTANCE && armTOF.isRangeValid() && robotController.isArmInPoleState();
        if (hasFoundReefPole) {
            detectedPole = true;
        }
        return hasFoundReefPole || detectedPole;
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
        return switch ((DriverStates) getCurrentState()) {
            case CENTERING_RIGHT, CENTERING_LEFT -> hasFinishedCentering();
            case PATH -> hasReachedTargetPose();
            case IDLE, DRIVER -> false;
        };
    }

    public enum DriverStates implements SubsystemState {
        IDLE,
        DRIVER,
        CENTERING_RIGHT,
        CENTERING_LEFT,
        PATH
    }
}
