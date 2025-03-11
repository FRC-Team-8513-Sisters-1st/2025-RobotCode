package frc.robot.Logic;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;

public class Vision {
    Robot thisRobot;

    public boolean useProcessorCam = true;
    boolean useLowerRightReefCam = true;
    boolean useCoralStationCam = true;
    boolean useLowerLeftReefCam = true;

    public double visionMaxATDist = Settings.maxATDistDisabeled;

    PhotonCamera processorCam = new PhotonCamera("processorCam");
    PhotonCamera lowerRightReefCam = new PhotonCamera("lowerRightReefCam");
    PhotonCamera coralStationCam = new PhotonCamera("coralStationCam");
    PhotonCamera lowerLeftReefCam = new PhotonCamera("lowerLeftReefCam");

    Matrix<N3, N1> visionSTDNoRotation = VecBuilder.fill(0.9, 0.9, 5);
    public boolean updateHeadingWithVision = true;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    Transform3d processorCamTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(7.3), Units.inchesToMeters(33.5)),
            new Rotation3d(0, Units.degreesToRadians(-26.5), Units.degreesToRadians(0)));
    Transform3d lowerRightReefCamTransorm = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(-10.5), Units.inchesToMeters(10)),
            new Rotation3d(0, 0, Units.degreesToRadians(11.2)));
    Transform3d coralStationCamTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(2.3), Units.inchesToMeters(7.3), Units.inchesToMeters(38)),
            new Rotation3d(0, Units.degreesToRadians(-26.5), Units.degreesToRadians(180)));
    Transform3d lowerLeftReefCamTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(7), Units.inchesToMeters(10)),
            new Rotation3d(0, 0, 0));

    PhotonPoseEstimator processorPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, processorCamTransform);
    PhotonPoseEstimator lowerRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowerRightReefCamTransorm);
    PhotonPoseEstimator coralStationEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, coralStationCamTransform);
    PhotonPoseEstimator lowerLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowerLeftReefCamTransform);

    Field2d photonField2d_processor = new Field2d();
    Field2d photonField2d_coralStation = new Field2d();
    Field2d photonField2d_lowerLeft = new Field2d();
    Field2d photonField2d_lowerRight = new Field2d();

    public Vision(Robot thisRobotIn) {
        thisRobot = thisRobotIn;
        SmartDashboard.putData("photonPose Processor", photonField2d_processor);
        SmartDashboard.putData("photonPose CoralStation", photonField2d_coralStation);
        SmartDashboard.putData("photonPose Lower Left", photonField2d_lowerLeft);
        SmartDashboard.putData("photonPose Lower Right", photonField2d_lowerRight);

    }

    public void updatePhotonVision() {

        integrateCamera(useProcessorCam, processorCam, processorPoseEstimator, photonField2d_processor,
                visionMaxATDist);
        integrateCamera(useLowerRightReefCam, lowerRightReefCam, lowerRightPoseEstimator, photonField2d_lowerRight,
                visionMaxATDist);
        integrateCamera(useLowerLeftReefCam, lowerLeftReefCam, lowerLeftPoseEstimator, photonField2d_lowerLeft,
                visionMaxATDist);
        integrateCamera(useCoralStationCam, coralStationCam, coralStationEstimator, photonField2d_coralStation,
                visionMaxATDist);
    }

    public void integrateCamera(boolean useCamera, PhotonCamera camera, PhotonPoseEstimator estimator,
            Field2d photonField, double maxDistance) {
        List<PhotonPipelineResult> cameraPipeline = camera.getAllUnreadResults();

            for (int i = 0; i < cameraPipeline.size(); i++) {
                Optional<EstimatedRobotPose> photonPose = estimator.update(cameraPipeline.get(i));
                if (photonPose.isPresent()) {
                    photonField.setRobotPose(photonPose.get().estimatedPose.toPose2d());
                    double tag0Dist = cameraPipeline.get(i).getBestTarget().bestCameraToTarget.getTranslation()
                            .getNorm();
                    double poseAmbaguitiy = cameraPipeline.get(i).getBestTarget().getPoseAmbiguity();
                    if (useCamera && tag0Dist < maxDistance && poseAmbaguitiy < 0.05) {

                        if (updateHeadingWithVision) {
                            thisRobot.drivebase.swerveDrive.addVisionMeasurement(
                                    photonPose.get().estimatedPose.toPose2d(),
                                    photonPose.get().timestampSeconds);
                        } else {
                            thisRobot.drivebase.swerveDrive.addVisionMeasurement(
                                    photonPose.get().estimatedPose.toPose2d(),
                                    photonPose.get().timestampSeconds, visionSTDNoRotation);
                        }
                    }
                }
            }
        }
    }
