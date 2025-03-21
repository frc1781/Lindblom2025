package tech.lindblom.subsystems.vision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
import static tech.lindblom.utils.EnumCollection.OperatingMode.AUTONOMOUS;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;

public class Vision extends Subsystem {
    private final RobotController robotController;

    private final PhotonCamera frontRightCamera = new PhotonCamera(Constants.Vision.FRONT_RIGHT_CAMERA_NAME);
    private PhotonPoseEstimator frontRightCameraPoseEstimator;
    private PhotonPipelineResult frontRightCameraPipelineResult;

    private final PhotonCamera frontLeftCamera = new PhotonCamera(Constants.Vision.FRONT_LEFT_CAMERA_NAME);
    private PhotonPoseEstimator frontLeftCameraPoseEstimator;
    private PhotonPipelineResult frontLeftCameraPipelineResult;

/*    private final PhotonCamera backCamera = new PhotonCamera(Constants.Vision.BACK_CAMERA_NAME);
    private PhotonPoseEstimator backCameraPoseEstimator;
    private PhotonPipelineResult backCameraPipelineResult;*/

    private final PhotonCamera leftSideCamera = new PhotonCamera(Constants.Vision.LEFT_SIDE_CAMERA_NAME);
    private PhotonPoseEstimator leftSideCameraPoseEstimator;
    private PhotonPipelineResult leftSideCameraPipelineResult;
    private Timer invalidTagTime;
    private int lastClosestAprilTag;

    VisionSystemSim visionSystemSim;

    private final List<Integer> reefApriltagIds = List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11);
    private List<Pose3d> seenAprilTags;

    private AprilTagFieldLayout fieldLayout;

    public Vision(RobotController _robotController) {
        super("Vision");
        this.robotController = _robotController;
        seenAprilTags = new ArrayList<>();
        invalidTagTime = new Timer();
        lastClosestAprilTag = -1;
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

            if (RobotBase.isSimulation()) {
                visionSystemSim = new VisionSystemSim("main");
                visionSystemSim.addAprilTags(fieldLayout);

                SimCameraProperties cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
                cameraProp.setCalibError(0.25, 0.08);
                cameraProp.setFPS(120);
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);
                PhotonCameraSim frontRightCameraSim = new PhotonCameraSim(frontRightCamera, cameraProp);
                PhotonCameraSim frontLeftCameraSim = new PhotonCameraSim(frontLeftCamera, cameraProp);
                PhotonCameraSim leftSideCameraSim = new PhotonCameraSim(leftSideCamera, cameraProp);

                visionSystemSim.addCamera(frontRightCameraSim, Constants.Vision.FRONT_RIGHT_CAMERA_POSITION);
                visionSystemSim.addCamera(frontLeftCameraSim, Constants.Vision.FRONT_LEFT_CAMERA_POSITION);
                visionSystemSim.addCamera(leftSideCameraSim, Constants.Vision.LEFT_SIDE_CAMERA_POSITION);

                Constants.Auto.AUTONOMOUS_TAB.add(visionSystemSim.getDebugField());
            }

            frontRightCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    Constants.Vision.FRONT_RIGHT_CAMERA_POSITION);
            frontLeftCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    Constants.Vision.FRONT_LEFT_CAMERA_POSITION);
            leftSideCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    Constants.Vision.LEFT_SIDE_CAMERA_POSITION);

            frontRightCameraPoseEstimator.setMultiTagFallbackStrategy(LOWEST_AMBIGUITY);
            frontLeftCameraPoseEstimator.setMultiTagFallbackStrategy(LOWEST_AMBIGUITY);
            leftSideCameraPoseEstimator.setMultiTagFallbackStrategy(LOWEST_AMBIGUITY);
        } catch (Exception e) {
            System.out.println("Could not initialize Vision, please view the error below.");
            System.out.println(e);
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            visionSystemSim.update(robotController.driveController.getRobotPose());
        }

        frontRightCameraPipelineResult = updatePhotonPoseEstimator(frontRightCameraPoseEstimator, frontRightCamera);
        frontLeftCameraPipelineResult = updatePhotonPoseEstimator(frontLeftCameraPoseEstimator, frontLeftCamera);
        leftSideCameraPipelineResult = updatePhotonPoseEstimator(leftSideCameraPoseEstimator, leftSideCamera);
        Logger.recordOutput(this.name + "/seenApriltags", seenAprilTags.toArray(Pose3d[]::new));
        seenAprilTags = new ArrayList<>();
    }

    public void forgetClosestAprilTag() {
        lastClosestAprilTag = -1;
    }

    public int getClosestReefApriltag(Camera camera) {
        PhotonPipelineResult result = getCameraLatestResults(camera);
        if (result == null || !result.hasTargets()) {
            if (!invalidTagTime.isRunning()) {
                invalidTagTime.start();
            } 
            return lastClosestAprilTag; //could be -1 if none seen recently, otherwise report the april tag we have been seeing
        } 
        else {
            invalidTagTime.reset();
        }

        List<PhotonTrackedTarget> targets = result.targets;
        PhotonTrackedTarget closestTarget = null;

        if (currentOperatingMode == AUTONOMOUS && robotController.autoSystem.getCurrentStep().hasTargetApriltag()) {
            int targetTag = robotController.autoSystem.getCurrentStep().getTargetApriltag();
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == targetTag) {
                    return targetTag;
                }
            }

            return -1;
        }

        for (PhotonTrackedTarget target : targets) {
            if (closestTarget == null && reefApriltagIds.contains(target.getFiducialId())) {
                closestTarget = target;
                continue;
            } else if (closestTarget == null) {
                continue;
            }

            if ((Math.abs(target.skew) < Math.abs(closestTarget.skew)) && reefApriltagIds.contains(target.getFiducialId())) {
                closestTarget = target;
            }
        }

        if (closestTarget == null || (closestTarget.getFiducialId() != lastClosestAprilTag && lastClosestAprilTag != -1)) {
            if (!invalidTagTime.isRunning()) {
                invalidTagTime.start();
            } 
        } 
        else {
            invalidTagTime.reset();
            lastClosestAprilTag = closestTarget.getFiducialId();
        }

        if (invalidTagTime.get() > 0.1) {
            lastClosestAprilTag = -1;  //have not seen one for a while
        }

        return lastClosestAprilTag;
    }

    public int getDoubleCameraReefApriltag() {
        PhotonPipelineResult leftResult = getCameraLatestResults(Camera.FRONT_LEFT);
        PhotonPipelineResult rightResult = getCameraLatestResults(Camera.FRONT_RIGHT);
        if (leftResult == null || rightResult == null || !leftResult.hasTargets() || !rightResult.hasTargets()) return -1;

        HashSet<Integer> leftTargets = new HashSet<>();
        for (PhotonTrackedTarget target : leftResult.targets) {
            leftTargets.add(target.getFiducialId());
        }
        ArrayList<Integer> rightTargets = new ArrayList<>();
        for (PhotonTrackedTarget target : rightResult.targets) {
            rightTargets.add(target.getFiducialId());
        }

        for (Integer target : rightTargets) {
            if (reefApriltagIds.contains(target) && leftTargets.contains(target)) {
                return target;
            }
        }

        return -1;
    }

    public double getCameraYaw(Camera camera, int tagID) {
        PhotonPipelineResult result = getCameraLatestResults(camera);

        if (result == null) {
            return Constants.Vision.ERROR_CONSTANT;
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagID) {
                return target.yaw;
            }
        }

        return Constants.Vision.ERROR_CONSTANT;
    }

    public double getCameraDistanceX(Camera camera, int tagID) {
        PhotonPipelineResult result = getCameraLatestResults(camera);
        if (result == null) {
            return Constants.Vision.ERROR_CONSTANT;
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagID) {
                return target.bestCameraToTarget.getX();
            }
        }

        return Constants.Vision.ERROR_CONSTANT;
    }

    public PhotonPipelineResult getCameraLatestResults(Camera camera) {
        PhotonPipelineResult result = null;
        switch (camera) {
/*            case BACK:
                result = backCameraPipelineResult;
                break;*/
            case FRONT_LEFT:
                result = frontLeftCameraPipelineResult;
                break;
            case FRONT_RIGHT:
                result = frontRightCameraPipelineResult;
                break;
            case LEFT_SIDE:
                result = leftSideCameraPipelineResult;
                break;
        }

        return result;
    }

    public PhotonPipelineResult updatePhotonPoseEstimator(PhotonPoseEstimator poseEstimator, PhotonCamera camera) {
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        if (!unreadResults.isEmpty() && camera.isConnected()) {
            for (PhotonPipelineResult result : unreadResults) {
                for (PhotonTrackedTarget target : result.targets) {
                    Optional<Pose3d> apriltagPose = fieldLayout.getTagPose(target.getFiducialId());
                    apriltagPose.ifPresent(seenAprilTags::add);
                }
                Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);

                if (estimatedRobotPose.isPresent()) {
                    updateRobotPose(result, estimatedRobotPose.get());
                }
            }
            return unreadResults.get(0);
        }

        return null;
    }

    public void updateRobotPose(PhotonPipelineResult result, EstimatedRobotPose robotPose) {
        if (robotPose.strategy == LOWEST_AMBIGUITY) {
            Rotation2d estimatedRotation = robotPose.estimatedPose.getRotation().toRotation2d();
            Rotation2d currentRotation = robotController.getRobotHeading();
            double rotationPercentError = Math.abs(((Math.abs(currentRotation.minus(estimatedRotation).getDegrees())) / (currentRotation.getDegrees() + 0.1)));
            Logger.recordOutput(this.name + "/singleTagPercentageError", rotationPercentError);
            if (rotationPercentError < 5.0) {
                this.robotController.updateLocalization(robotPose, result);
            }
        } else if (robotPose.strategy == MULTI_TAG_PNP_ON_COPROCESSOR) {
            this.robotController.updateLocalization(robotPose, result);
        }
    }

    // COMPLETELY TAKEN FROM 7525. THANK YOU SO MUCH.
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
        var estStdDevs = Constants.Vision.SINGLE_STD; // automatically assume one tag seen MIGHT BE SLOWER THAN GETTING
        // PIPELINE VALUE AND DECIDING????????
        var targets = pipelineResult.getTargets();
        int numTags = 0; // MIGHT BE SLOWER THAN GETTING PIPELINE VALUE AND DECIDING????????
        double avgDist = 0;
        double avgWeight = 0;
        for (var itag : targets) { // goes through all tags and calculates distance away from the target to bot
            var tagPose = fieldLayout.getTagPose(itag.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation()); // distance from bot to what tag should be

            avgWeight += Constants.Vision.TAG_WEIGHTS[itag.getFiducialId() - 1];
        }
        if (numTags == 0)
            return estStdDevs; // if you don't see don't change/keep the normal one

        avgDist /= numTags; // making it an average
        avgWeight /= numTags; // making it an average

        if (numTags > 1) {
            estStdDevs = Constants.Vision.MULTI_STD; // more trust in vision if mutliple tags
        }
        if (numTags == 1 && avgDist > Constants.Vision.STD_TRUSTABLE_DISTANCE) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); // less trust in vision
            // if one tag
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30)); // random goofy numbers from other team? but
            // basically gets weight from distance so
            // distance important but we have consistent i
            // think
        }

        estStdDevs = estStdDevs.times(avgWeight); // dynamic portion where matrix is updated based on how confident we
        // are in the tags we can see

        return estStdDevs;
    }

    public enum Camera {
        FRONT_RIGHT, FRONT_LEFT, BACK, LEFT_SIDE
    }
}
