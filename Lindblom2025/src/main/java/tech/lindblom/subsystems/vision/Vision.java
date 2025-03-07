package tech.lindblom.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;

import java.util.*;

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

    private final List<Integer> reefApriltagIds = List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11);

    private AprilTagFieldLayout fieldLayout;

    public Vision(RobotController _robotController) {
        super("Vision");
        this.robotController = _robotController;
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            frontRightCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    Constants.Vision.FRONT_RIGHT_CAMERA_POSITION);
            frontLeftCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    Constants.Vision.FRONT_LEFT_CAMERA_POSITION);
/*            backCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    Constants.Vision.BACK_CAMERA_POSITION);*/
            leftSideCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    Constants.Vision.LEFT_SIDE_CAMERA_POSITION);

            frontRightCameraPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
            frontLeftCameraPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
            //backCameraPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            leftSideCameraPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
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
        frontRightCameraPoseEstimator.addHeadingData(edu.wpi.first.wpilibj.RobotController.getFPGATime(), robotController.getRobotHeading());
        frontLeftCameraPoseEstimator.addHeadingData(edu.wpi.first.wpilibj.RobotController.getFPGATime(), robotController.getRobotHeading());
        leftSideCameraPoseEstimator.addHeadingData(edu.wpi.first.wpilibj.RobotController.getFPGATime(), robotController.getRobotHeading());
        //backCameraPoseEstimator.addHeadingData();

        frontRightCameraPipelineResult = updatePhotonPoseEstimator(frontRightCameraPoseEstimator, frontRightCamera);
        frontLeftCameraPipelineResult = updatePhotonPoseEstimator(frontLeftCameraPoseEstimator, frontLeftCamera);
        leftSideCameraPipelineResult = updatePhotonPoseEstimator(leftSideCameraPoseEstimator, leftSideCamera);
        //backCameraPipelineResult = updatePhotonPoseEstimator(backCameraPoseEstimator, backCamera);
    }

    public int getClosestReefApriltag(Camera camera) {
        PhotonPipelineResult result = getCameraLatestResults(camera);
        if (result == null || !result.hasTargets()) return -1;
        List<PhotonTrackedTarget> targets = result.targets;
        PhotonTrackedTarget closestTarget = null;
        for (PhotonTrackedTarget target : targets) {
            if (closestTarget == null && reefApriltagIds.contains(target.getFiducialId())) {
                closestTarget = target;
                continue;
            } else if (closestTarget == null) {
                continue;
            }

            if ((target.area < closestTarget.area) && reefApriltagIds.contains(target.getFiducialId())) {
                closestTarget = target;
            }
        }

        return closestTarget == null ? -1 : closestTarget.getFiducialId();
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

    public double getCameraSkew(Camera camera, int tagID) {
        PhotonPipelineResult result = getCameraLatestResults(camera);

        if (result == null) {
            return Constants.Vision.ERROR_CONSTANT;
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagID) {
                return target.bestCameraToTarget.getRotation().getAngle();
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
        List<PhotonTrackedTarget> seenAprilTags = new ArrayList();
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        if (!unreadResults.isEmpty() && camera.isConnected()) {
            for (PhotonPipelineResult result : unreadResults) {
                seenAprilTags.addAll(result.targets);
                Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
                estimatedRobotPose.ifPresent(robotPose -> this.robotController.updateLocalization(robotPose, result));
            }

            return unreadResults.get(0);
        }

        return null;
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
