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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;

public class Vision {
    Robot thisRobot;

    boolean useProcessorCam = true;
    boolean useLowerRightReefCamm = true;
    boolean useCoralStationCam = true;
    boolean useLowerLeftReefCam = true;

    PhotonCamera processorCam = new PhotonCamera("processorCam");
    PhotonCamera lowerRightReefCam = new PhotonCamera("lowerRightReefCam");
    PhotonCamera coralStationCam = new PhotonCamera("coralStationCam");
    PhotonCamera lowerLeftReefCam = new PhotonCamera("lowerLeftReefCam");

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Transform3d processorCamTransform = new Transform3d(new Translation3d(Units.inchesToMeters(4), Units.inchesToMeters(-11), Units.inchesToMeters(33)),
            new Rotation3d(0, Units.degreesToRadians(-24), 0));
    Transform3d lowerRightReefCamTransorm = new Transform3d(new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(-10.5), Units.inchesToMeters(10)),
            new Rotation3d(0, 0, Units.degreesToRadians(20)));
    Transform3d coralStationCamTransform = new Transform3d(new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(8.5), Units.inchesToMeters(38)),
            new Rotation3d(0, Units.degreesToRadians(-27), Units.degreesToRadians(180)));
    Transform3d lowerLeftReefCamTransform = new Transform3d(new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(7), Units.inchesToMeters(10)),
            new Rotation3d(0, Units.degreesToRadians(5), 0));

    PhotonPoseEstimator processorPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, processorCamTransform);
    PhotonPoseEstimator lowerRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowerRightReefCamTransorm);
    PhotonPoseEstimator coralStationEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, coralStationCamTransform);
    PhotonPoseEstimator lowerLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowerLeftReefCamTransform);

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
                double tag0Dist = photonUpdateProcessorCam.get(0).getBestTarget().bestCameraToTarget.getTranslation().getNorm();
                if (useProcessorCam && tag0Dist < Settings.maxATDist) {
                    thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(),
                            optPhotonPose.get().timestampSeconds);
                }
            }
        }

        List<PhotonPipelineResult> photonUpdateLowerRightReefCam = lowerRightReefCam.getAllUnreadResults();
        if (photonUpdateLowerRightReefCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = lowerRightPoseEstimator
                    .update(photonUpdateLowerRightReefCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("lowerRightReefCam", photonField2d);
                double tag0Dist = photonUpdateLowerRightReefCam.get(0).getBestTarget().bestCameraToTarget.getTranslation().getNorm();
                if (useLowerRightReefCamm && tag0Dist < Settings.maxATDist) {
                    thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(),
                            optPhotonPose.get().timestampSeconds);

                }
            }
        }

        List<PhotonPipelineResult> photonUpdateCoralStationCam = coralStationCam.getAllUnreadResults();
        if (photonUpdateCoralStationCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = coralStationEstimator
                    .update(photonUpdateCoralStationCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("coralStationCam", photonField2d);
                double tag0Dist = photonUpdateCoralStationCam.get(0).getBestTarget().bestCameraToTarget.getTranslation().getNorm();
                if (useCoralStationCam && tag0Dist < Settings.maxATDist) {
                    thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(),
                            optPhotonPose.get().timestampSeconds);
                }
            }
        }

        List<PhotonPipelineResult> photonUpdateLowerLeftReefCam = lowerLeftReefCam.getAllUnreadResults();
        if (photonUpdateLowerLeftReefCam.size() > 0) {
            Optional<EstimatedRobotPose> optPhotonPose = lowerLeftPoseEstimator
                    .update(photonUpdateLowerLeftReefCam.get(0));
            if (optPhotonPose.isPresent()) {
                photonField2d.setRobotPose(optPhotonPose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("lowerLeftReefCam", photonField2d);
                double tag0Dist = photonUpdateLowerLeftReefCam.get(0).getBestTarget().bestCameraToTarget.getTranslation().getNorm();
                if (useLowerLeftReefCam && tag0Dist < Settings.maxATDist) {
                    thisRobot.drivebase.swerveDrive.addVisionMeasurement(optPhotonPose.get().estimatedPose.toPose2d(),
                            optPhotonPose.get().timestampSeconds);
                }
            }
        }
    }
}
