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
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;

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
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera,
                    Constants.Vision.frontCameraPositionOnRobot);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        frontCameraRobotPose = frontCameraPoseEstimator.update();
        areAprilTagsDetected = frontCameraRobotPose.isPresent();;

        if (areAprilTagsDetected) {
            Logger.recordOutput(this.name + "/FrontCameraEstimatedPose", frontCameraRobotPose.get().estimatedPose);
        }
    }

    public Optional<Pose2d> getFrontCameraPose() {
        if (frontCameraRobotPose.isPresent()) {
            return Optional.of(frontCameraRobotPose.get().estimatedPose.toPose2d());
        }

        return Optional.empty();
    }

    public PhotonPipelineResult getFrontCameraPipelineResult() {
        return frontCamera.getLatestResult();
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
