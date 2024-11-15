package tech.lindblom.subsystems.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.targeting.PhotonPipelineResult;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.EnumCollection;

import java.util.Optional;

import static tech.lindblom.utils.EnumCollection.OperatingMode.*;

public class DriveController extends Subsystem {
    private final Drive driveSubsystem;
    private final RobotController robotController;
    private PathPlannerPath mFollowingPath;
    private HolonomicDriveController mTrajectoryController;

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
        mFollowingPath = path; //The path we are following
        ChassisSpeeds desiredChassisSpeeds; // The ChassisSpeeds generated from the following trajectory
        //Start timer
        //depending on the timer, look at what states the trajectory has to be in
        //convert the specific state to chassisspeeds to follow the path

        // mFollowingPath.getTrajectory(mTrajectoryController.calculate(driveSubsystem.getRobotPose(), null, null), driveSubsystem.getRobotRotation())


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
                driveSubsystem.setInitialPose(visionPose);
                return;
            }
        }

        if (mode == AUTONOMOUS) {
            try {
                Pose2d poseFromPath = robotController.autoSystem.getStartPosition();
                driveSubsystem.setInitialPose(poseFromPath);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (mode == TELEOP) {
            double startingDegRotation = RobotController.isRed() ? 180 : 0;
            driveSubsystem.setInitialPose(new Pose2d(new Translation2d(), new Rotation2d(startingDegRotation)));
        }
    }
}
