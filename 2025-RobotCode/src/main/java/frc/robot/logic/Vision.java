package frc.robot.logic;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class Vision {
    Robot thisRobot;

    PhotonCamera cam = new PhotonCamera("testCamera");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    Field2d photonField2d = new Field2d();
    
    public Vision(Robot thisRobotIn) {
        thisRobot = thisRobotIn;
    }

    public void updatePhotonVision() {
        List<PhotonPipelineResult> photonUpdate = cam.getAllUnreadResults();
        if (photonUpdate.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = photonPoseEstimator.update(photonUpdate.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("photon pose", photonField2d);
                thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(), optPhotonPose.get().timestampSeconds);
            }
        }
    }

}
