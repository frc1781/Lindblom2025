package tech.lindblom.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Vision extends Subsystem {
    private Optional<EstimatedRobotPose> frontCameraRobotPose = Optional.empty();

    private PhotonCamera frontCamera = new PhotonCamera(Constants.Vision.FrontCameraName);
    private PhotonPoseEstimator frontCameraPoseEstimator;

    private AprilTagFieldLayout fieldLayout;

    private boolean areAprilTagsDetected = false;

    public Vision() {
        super("Vision");
        try {
            String deployDirectoryPath = Filesystem.getDeployDirectory().getAbsolutePath();
            fieldLayout = new AprilTagFieldLayout(deployDirectoryPath + "/CrescendoFieldLayout.json");
            frontCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    Constants.Vision.frontCameraPositionOnRobot);
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
        List<PhotonPipelineResult> unreadResults = frontCamera.getAllUnreadResults();
        if (!unreadResults.isEmpty()) {
            frontCameraRobotPose = frontCameraPoseEstimator.update(unreadResults.get(0));
        }

        areAprilTagsDetected = frontCameraRobotPose.isPresent();

        Logger.recordOutput(this.name + "/FrontCamera/AprilTagsDetected", areAprilTagsDetected);
        if (areAprilTagsDetected) {
            Logger.recordOutput(this.name + "/FrontCamera/EstimatedPose", frontCameraRobotPose.get().estimatedPose.toPose2d());
            List<PhotonTrackedTarget> targetList = frontCameraRobotPose.get().targetsUsed;
            Pose2d[] targetPoses = new Pose2d[targetList.size()];
            for (int i = 0; i < targetList.size(); i++) {
                PhotonTrackedTarget target = targetList.get(i);
                targetPoses[i] = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
            }

            Logger.recordOutput(this.name + "/FrontCamera/TargetsUsed", targetPoses);
        }
    }

    public Optional<Pose2d> getFrontCameraPose() {
        return frontCameraRobotPose.map(estimatedRobotPose -> estimatedRobotPose.estimatedPose.toPose2d());

    }

    public PhotonPipelineResult getFrontCameraPipelineResult() {
        List<PhotonPipelineResult> unreadResults = frontCamera.getAllUnreadResults();
        if (!unreadResults.isEmpty()) {
            frontCameraRobotPose = frontCameraPoseEstimator.update(unreadResults.get(0));
        }
        return new PhotonPipelineResult();
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
}
