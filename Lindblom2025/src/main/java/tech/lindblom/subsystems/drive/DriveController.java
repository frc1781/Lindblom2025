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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.control.DriverInput.ReefCenteringSide;
import tech.lindblom.subsystems.led.LEDs.LEDState;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.subsystems.vision.Vision.Camera;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EEtimeOfFlight;
import tech.lindblom.utils.EnumCollection;

import java.util.HashMap;

import static tech.lindblom.utils.EnumCollection.OperatingMode.*;

public class DriveController extends StateSubsystem {
    private final Drive driveSubsystem;
    private final RobotController robotController;
    private PathPlannerPath followingPath;
    private PathPlannerTrajectory followingTrajectory;
    private final Timer trajectoryTime;
    private Pose2d targetPose;
    private HolonomicDriveController trajectoryController;
    private final EEtimeOfFlight leftTOF;
    private final EEtimeOfFlight rightTOF;
    private final EEtimeOfFlight armTOF;

    private final PIDController XController = new PIDController(3, 0, 0);
    private final PIDController YController = new PIDController(3, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(4, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));
    private final ProfiledPIDController centeringRotController = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));

    private final PIDController centeringYawController = new PIDController(0.035, 0, 0);
    private final PIDController distanceController = new PIDController(1, 0, 0);
    private final ProfiledPIDController parallelController = new ProfiledPIDController(0.1, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));

    private final HashMap<Integer, Double> reefApriltagAngle = new HashMap<>();

    private RobotConfig robotConfig;
    private final boolean isFieldOriented = true;
    private boolean reachedDesiredDistance = false;
    private boolean detectedPole = false;
    private boolean hasSetInitialPose = false;
    private int targetedCenteringApriltagId = -1;

    public DriveController(RobotController controller) {
        super("DriveController", DriverStates.IDLE);
        driveSubsystem = new Drive();
        robotController = controller;
        armTOF = new EEtimeOfFlight(Constants.Drive.ARM_TOF_ID, 20);
        armTOF.tof.setRangeOfInterest(6, 6, 10, 10);
        leftTOF = new EEtimeOfFlight(Constants.Drive.LEFT_FRONT_TOF_ID, 20);
        rightTOF = new EEtimeOfFlight(Constants.Drive.RIGHT_FRONT_TOF_ID, 20);

        centeringRotController.enableContinuousInput(0, Math.PI * 2);
        rotController.enableContinuousInput(0, Math.PI * 2);
        parallelController.enableContinuousInput(0, Math.PI * 2);
        centeringRotController.enableContinuousInput(-180, 180);
        trajectoryTime = new Timer();

        reefApriltagAngle.put(17, 60.0);
        reefApriltagAngle.put(18, 0.0);
        reefApriltagAngle.put(19, -60.0);
        reefApriltagAngle.put(20, -120.0);
        reefApriltagAngle.put(21, 180.0);
        reefApriltagAngle.put(22, 120.0);

        reefApriltagAngle.put(6, 120.0);
        reefApriltagAngle.put(7, 180.0);
        reefApriltagAngle.put(8, -120.0);
        reefApriltagAngle.put(9, -60.0);
        reefApriltagAngle.put(10, 0.0);
        reefApriltagAngle.put(11, 60.0);
    }

    @Override
    public void init() {
        super.init();
        centeringYawController.reset();
        distanceController.reset();
        parallelController.reset(driveSubsystem.getRobotPose().getRotation().getRadians());

        switch (currentOperatingMode) {
            case DISABLED:
                break;
            case AUTONOMOUS:
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
        Logger.recordOutput(this.name + "/armTOFVaild", armTOF.isRangeValidRegularCheck());
        Logger.recordOutput(this.name + "/leftTOF", leftTOF.getRange());
        Logger.recordOutput(this.name + "/leftTOFVaild", leftTOF.isRangeValidRegularCheck());
        Logger.recordOutput(this.name + "/rightTOF", rightTOF.getRange());
        Logger.recordOutput(this.name + "/rightTOFVaild", rightTOF.isRangeValidRegularCheck());
        if (trajectoryTime != null) {
            Logger.recordOutput(this.name + "/trajectoryTime", trajectoryTime.get());
        }
        Logger.recordOutput(this.name + "/hasSetInitialPose", hasSetInitialPose);

        if (currentOperatingMode == DISABLED) return;

        switch ((DriverStates) getCurrentState()) {
            case IDLE:
                driveSubsystem.drive(zeroSpeed());
                break;
            case CENTERING_LEFT, CENTERING_RIGHT, CENTERING_CENTER:
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

    public void orientFieldToRobot() {
        driveSubsystem.orientFieldToRobot();
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
                driveSubsystem.getRobotRotation())
                : new ChassisSpeeds(xVelocity, yVelocity, rotSpeed);

        driveSubsystem.drive(speeds);
    }

    private boolean isValidCameraReading(double cameraOffset) {
       return 
        cameraOffset != Constants.Vision.ERROR_CONSTANT &&
        cameraOffset != 0.0; //0.0 indicates it is not estimating distance
    }

    public boolean readyForCentering() {
         return leftTOF.isRangeValidRegularCheck() && rightTOF.isRangeValidRegularCheck() && 
            leftTOF.getRange() < 1000 && rightTOF.getRange() < 1000 && (
                robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_LEFT) != -1 || 
                robotController.visionSystem.getClosestReefApriltag(Vision.Camera.FRONT_RIGHT) != 1
            );
    }

    public ChassisSpeeds getCenteringChassisSpeeds(ChassisSpeeds inputSpeeds) {
        if (robotController.getCenteringSide() == null) 
            return inputSpeeds;

        // hi ian - ally 3/20/25
        int apriltagId = 0;
        double cameraOffset = 0.0;
        double targetOffset;
        double targetParallelDistance = Constants.Drive.TARGET_TOF_PARALLEL_DISTANCE;

        if (targetedCenteringApriltagId == -1) {
            targetedCenteringApriltagId = robotController.visionSystem.getDoubleCameraReefApriltag(); // could return -1 . make led state for that
        }

        apriltagId = targetedCenteringApriltagId;

        if (targetedCenteringApriltagId == -1) {
            return inputSpeeds;
        }

        if (robotController.getCurrentSequentialAction() != null)  {
            switch (robotController.getCurrentSequentialAction()) {
                case L4:
                    break;
                case L3, L2:
                    targetParallelDistance = Constants.Drive.TARGET_TOF_PARALLEL_DISTANCE_SHORT;
                    break;
            }
        }
        
        switch (robotController.getCenteringSide()) {
            case LEFT:
                targetOffset = Constants.Drive.TARGET_CORAL_OFFSET_LEFT;
                cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_RIGHT, apriltagId);
                break;
            case RIGHT:
                targetOffset = Constants.Drive.TARGET_CORAL_OFFSET_RIGHT;
                cameraOffset = robotController.visionSystem.getCameraYaw(Vision.Camera.FRONT_LEFT, apriltagId);
                break;
            case CENTER:
                targetParallelDistance = Constants.Drive.TARGET_TOF_CENTERING_PARALLEL_DISTANCE;
                targetOffset = Constants.Drive.TARGET_CORAL_OFFSET_CENTER_CAMERA_1;
                cameraOffset = robotController.visionSystem.getCameraYaw(Camera.FRONT_RIGHT, apriltagId);
                break;
            default:
                return inputSpeeds;
        }

        if(robotController.getCenteringSide() != ReefCenteringSide.CENTER)
        {
            if (cameraOffset != Constants.Vision.ERROR_CONSTANT) {
                inputSpeeds.vyMetersPerSecond = EEUtil.clamp(-0.2, 0.2, centeringYawController.calculate(cameraOffset, targetOffset));
            } else if (robotController.isElevatorInPoleState() && robotController.isArmInPoleState()) {
                inputSpeeds.vyMetersPerSecond = EEUtil.clamp(-0.3, 0.3, getCurrentState() == DriverStates.CENTERING_RIGHT ? -0.30 : 0.30);
            }
        }

        if (robotController.getCenteringSide() == ReefCenteringSide.CENTER && cameraOffset != Constants.Vision.ERROR_CONSTANT) {
            inputSpeeds.vyMetersPerSecond = centeringYawController.calculate(cameraOffset, targetOffset);
        }

        if (EEUtil.angleDiffDegrees(getRobotHeading().getDegrees(), reefApriltagAngle.get(apriltagId)) > 1) {
            System.out.println(getRobotHeading().getRadians() + " " + Rotation2d.fromDegrees(reefApriltagAngle.get(apriltagId)).getRadians());
            inputSpeeds.omegaRadiansPerSecond = EEUtil.clamp(-0.5, 0.5, centeringRotController.calculate(getRobotHeading().getRadians(), Rotation2d.fromDegrees(reefApriltagAngle.get(apriltagId)).getRadians()));
        } else {
            inputSpeeds.omegaRadiansPerSecond = 0;
        }

        double leftTOFDistance = leftTOF.getRange();
        double rightTOFDistance = rightTOF.getRange();
        if (leftTOF.isRangeValidRegularCheck() && rightTOF.isRangeValidRegularCheck()) {
            if (Math.abs((leftTOFDistance + rightTOFDistance) / 2.0 - targetParallelDistance) >= 50) {
                inputSpeeds.vxMetersPerSecond = EEUtil.clamp(-0.5, 0.5, 0.005 * ((leftTOFDistance + rightTOFDistance) / 2.0 - targetParallelDistance));
            } else {
                inputSpeeds.vxMetersPerSecond = 0;
            }

            if (/* inputSpeeds.omegaRadiansPerSecond <  && */ inputSpeeds.vxMetersPerSecond == 0) {
                reachedDesiredDistance = true;
            }

            if (Math.abs(rightTOFDistance - leftTOFDistance) > 250) {
                robotController.ledsSystem.setState(LEDState.OVER);
            }
        }

        Logger.recordOutput(this.name + "/parallelDistance", rightTOFDistance - leftTOFDistance);
        Logger.recordOutput(this.name + "/inCenteredPosition", reachedDesiredDistance);
        Logger.recordOutput(this.name + "/driveUsingVelocities", inputSpeeds);
        Logger.recordOutput(this.name + "/cameraOffset", cameraOffset);
        Logger.recordOutput(this.name + "/apriltagId", apriltagId);
        Logger.recordOutput(this.name + "/leftTOFDistance", leftTOF.getRange());
        Logger.recordOutput(this.name + "/rightTOFDistance", rightTOF.getRange());
        Logger.recordOutput(this.name + "/poleTOFdDistance", armTOF.getRange());
        Logger.recordOutput(this.name + "/hasFoundReefPole", hasFoundReefPole());
        Logger.recordOutput(this.name + "/centeringAprilTag", apriltagId);
        Logger.recordOutput(this.name + "/isElevatorInPoleState", robotController.isElevatorInPoleState());
        Logger.recordOutput(this.name + "/isArmInPoleState", robotController.isArmInPoleState());
        
        if (reachedDesiredDistance && hasFoundReefPole()) {
            if (robotController.isArmInL4()) {
                inputSpeeds.vxMetersPerSecond = -0.5;
                inputSpeeds.omegaRadiansPerSecond = 0;
                inputSpeeds.vyMetersPerSecond = 0;
                return inputSpeeds;
            }
            return zeroSpeed();
        }

        return inputSpeeds;
    }

    @Override
    public void stateTransition(SubsystemState previousState, SubsystemState newState) {
        if (newState == DriverStates.CENTERING_LEFT || 
            newState == DriverStates.CENTERING_RIGHT || 
            newState == DriverStates.CENTERING_CENTER) {  
            detectedPole = false;
            reachedDesiredDistance = false;
            robotController.visionSystem.forgetClosestAprilTag();
            targetedCenteringApriltagId = -1;
        }

        if (previousState == DriverStates.PATH) {
            trajectoryTime.stop();
            trajectoryTime.reset();
        }

        if (newState == DriverStates.PATH) {
            trajectoryTime.reset();
            trajectoryTime.start();
        }

        if (previousState == DriverStates.CENTERING_LEFT || previousState == DriverStates.CENTERING_RIGHT || previousState == DriverStates.CENTERING_CENTER) {
            targetedCenteringApriltagId = -1;
        }
    }

    public Rotation2d getRobotHeading() {
        return driveSubsystem.getRobotRotation();
    }

    public void centerOnReef() {
        if (robotController.getCenteringSide() == null) {
            return;
        }

        ChassisSpeeds centeringSpeeds = getCenteringChassisSpeeds(zeroSpeed());
        driveSubsystem.drive(centeringSpeeds);
    }

    public boolean isSafeForElevatorStage2toMove() {  //Apparently very experimental and should not be relied on, who knows ??
        return driveSubsystem.getGioSpeed() < 2;
    }

    public boolean hasFinishedCentering() {
        if (robotController.getCenteringSide() == null)  {
            return false;
        }

        return reachedDesiredDistance && hasFoundReefPole();
    }

    public boolean reefPoleDetected() {  //added just for LEDs
        return armTOF.getRange() < Constants.Drive.ARM_TOF_DISTANCE && armTOF.isRangeValidRegularCheck()  && robotController.armSystem.getPosition() < 60;
    }

    public boolean hasFoundReefPole() {
        boolean hasFoundReefPole = armTOF.getRange() < Constants.Drive.ARM_TOF_DISTANCE && armTOF.isRangeValidRegularCheck() && robotController.isArmInPoleState() && robotController.isElevatorInPoleState();
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
        trajectoryTime.reset();
    }

    public void followPath() {
        PathPlannerTrajectoryState pathplannerState = followingTrajectory.sample(trajectoryTime.get());
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
        if (targetPose == null) return false;

        if (Robot.isSimulation()) {
            return trajectoryTime.get() > followingTrajectory.getTotalTimeSeconds();
        }

        Pose2d currentPose = driveSubsystem.getRobotPose();
        boolean reachedTargetTranslation = currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1;
        boolean reachedTargetHeading = true;

        return (reachedTargetHeading && reachedTargetTranslation) || robotController.shouldBeCentering();
    }

    public void updatePoseUsingVisionEstimate(Pose2d estimatedPose, double time, Matrix<N3, N1> stdValue) {
        Logger.recordOutput(this.name + "/VisionEstimatedPose", estimatedPose);
        driveSubsystem.updatePoseUsingVisionEstimate(estimatedPose, time, stdValue);
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
 
        } else if (mode == TELEOP && !hasSetInitialPose) {
           driveSubsystem.setInitialPose(new Pose2d(new Translation2d(), new Rotation2d(0)));
        }
        
        hasSetInitialPose = true;
    }

    public Pose2d getRobotPose() {
        if (RobotBase.isSimulation()) {
            return new Pose2d(2, 2, new Rotation2d());
        }

        return driveSubsystem.getRobotPose();
    }

    @Override
    public boolean matchesState() {
        return switch ((DriverStates) getCurrentState()) {
            case CENTERING_RIGHT, CENTERING_LEFT, CENTERING_CENTER -> hasFinishedCentering();
            case PATH -> hasReachedTargetPose();
            case IDLE, DRIVER, INHIBIT_DRIVE -> false;
        };
    }

    public enum DriverStates implements SubsystemState {
        IDLE,
        DRIVER,
        INHIBIT_DRIVE,
        CENTERING_RIGHT,
        CENTERING_LEFT,
        CENTERING_CENTER,
        PATH
    }
}
