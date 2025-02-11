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

    PhotonCamera processorCam = new PhotonCamera("processorCam");
    PhotonCamera lowerRightReefCam = new PhotonCamera("lowerRightReefCam");
    PhotonCamera coralStationCam = new PhotonCamera("coralStationCam");
    PhotonCamera lowerLeftReefCam = new PhotonCamera("lowerLeftReefCam");

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Transform3d processorCamTransform = new Transform3d(new Translation3d(-21.483, 7.955, 26.681), new Rotation3d(0, 0, 0));
    Transform3d lowerRightReefCamTransorm = new Transform3d(new Translation3d(18.139, 9.098, 5), new Rotation3d(0, 0, 0));
    Transform3d coralStationCamTransform = new Transform3d(new Translation3d(17.983, -16.955, -26.681), new Rotation3d(0, 0, 0));
    Transform3d lowerLeftReefCamTransform = new Transform3d(new Translation3d(-21.733, 8.754, -10), new Rotation3d(0, 0, 0));

    PhotonPoseEstimator processorPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, processorCamTransform);
    PhotonPoseEstimator lowerRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowerRightReefCamTransorm);
    PhotonPoseEstimator coralStationEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, coralStationCamTransform);
    PhotonPoseEstimator lowerLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowerLeftReefCamTransform);

    Field2d photonField2d = new Field2d();
    
    public Vision(Robot thisRobotIn) {
        thisRobot = thisRobotIn;
    }

    public void updatePhotonVision() {
        List<PhotonPipelineResult> photonUpdateProcessorCam = processorCam.getAllUnreadResults();
        if (photonUpdateProcessorCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = processorPoseEstimator.update(photonUpdateProcessorCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("processorCam", photonField2d);
                thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(), optPhotonPose.get().timestampSeconds);
            }
        }

        List<PhotonPipelineResult> photonUpdateLowerRightReefCam = lowerRightReefCam.getAllUnreadResults();
        if (photonUpdateLowerRightReefCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = lowerRightPoseEstimator.update(photonUpdateLowerRightReefCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("lowerRightReefCam", photonField2d);
                thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(), optPhotonPose.get().timestampSeconds);
            }
        }

    List<PhotonPipelineResult> photonUpdateCoralStationCam = coralStationCam.getAllUnreadResults();
        if (photonUpdateCoralStationCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = coralStationEstimator.update(photonUpdateCoralStationCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("coralStationCam", photonField2d);
                thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(), optPhotonPose.get().timestampSeconds);
            }
        }

    List<PhotonPipelineResult> photonUpdateLowerLeftReefCam = lowerLeftReefCam.getAllUnreadResults();
        if (photonUpdateLowerLeftReefCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = lowerLeftPoseEstimator.update(photonUpdateLowerLeftReefCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("lowerLeftReefCam", photonField2d);
                thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(), optPhotonPose.get().timestampSeconds);
            }
        }
    }
}
