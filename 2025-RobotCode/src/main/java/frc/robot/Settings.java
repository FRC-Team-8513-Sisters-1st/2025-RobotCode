package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Settings {
        // drivebase settings
        public static double drivebaseMaxVelocityFPS = 17.1;

        // vision settings
        public static boolean useVision = true;

        // joystick deadband
        public static double joystickDeadband = 0.01;
        public static double triggerDeadband = 0.2;

        // driver axis
        public static int leftRightAxis = 0;
        public static int forwardBackwardsAxis = 1;
        public static int rotAxis = 4;

        // driver joystick settings
        public static int driverJoystickPort = 1;
        public static int operatorJoystick1Port = 2;
        public static int operatorJoystick2Port = 3;

        // pid settings
        public static double maxVelocityAP = 3;
        public static double maxAccelerationAP = 3;
        private static double attackPointDriveP = 5;
        private static double attackPointDriveI = 0.1;
        private static double attackPointDriveD = 0.01;
        private static double attackPointDt = 0.02;

        // rotation
        public static double maxVelocityRotateAP = 360;
        public static double maxAccelerationRotateAP = 360;
        private static double attackPointRotateP = 0.1;
        private static double attackPointRotateI = 0;
        private static double attackPointRotateD = 0;

        private static final TrapezoidProfile.Constraints attackPointConstraints = new TrapezoidProfile.Constraints(
                        maxVelocityAP,
                        maxAccelerationAP);
        public static final ProfiledPIDController xControllerAP = new ProfiledPIDController(attackPointDriveP,
                        attackPointDriveI,
                        attackPointDriveD,
                        attackPointConstraints, attackPointDt);
        public static final ProfiledPIDController yControllerAP = new ProfiledPIDController(attackPointDriveP,
                        attackPointDriveI,
                        attackPointRotateD,
                        attackPointConstraints, attackPointDt);

        // rotation
        private static final TrapezoidProfile.Constraints attackPointDriveConstraints = new TrapezoidProfile.Constraints(
                        maxVelocityRotateAP,
                        maxAccelerationRotateAP);
        public static final ProfiledPIDController rControllerAP = new ProfiledPIDController(attackPointRotateP,
                        attackPointRotateI,
                        attackPointRotateD,
                        attackPointDriveConstraints, attackPointDt);

        // pid path follow settings
        public static PIDController xController = new PIDController(3.5, 0.05, 0.01);
        public static PIDController yController = new PIDController(3.5, 0.05, 0.01);
        public static PIDController rController = new PIDController(0.1, 0, 0);

        public static PIDController rJoystickController = new PIDController(0.1, 0, 0);
        public static int rightJoystickY = 5;
        public static int rightJoystickX = 4;
        public static boolean headingJoystickControls = false;

        // CANids
        public static int algaeMotor1CANID = 62;
        public static int algaeGroundMotor1CANID = 13;
        public static int algaeGroundMotor2CANID = 14;
        public static int climberMotor1CANID = 45;
        public static int elevatorMotor1CANID = 59; // e2
        public static int elevatorMotor2CANID = 57; // e1
        public static int coralMotor1CANID = 55;
        public static int funnelMotor1 = 21;

        // controller 1 panel buttons
        public static int buttonId_CoralOutake = 5;
        public static int buttonId_CoralIntake = 9;
        public static int buttonId_Coral4 = 6;
        public static int buttonId_Coral3 = 7;
        public static int buttonId_Coral2 = 8;
        public static int buttonId_Coral1 = 4;
        public static int buttonId_Algae3 = 1;
        public static int buttonId_Algae2 = 2;
        public static int buttonId_processor = 3;

        // controller 2 panel buttons
        public static int buttonId_Close = 6;
        public static int buttonId_Far = 7;
        public static int buttonId_forceElevator = 5;
        public static int buttonId_ab = 9;
        public static int buttonId_cd = 10;
        public static int buttonId_ef = 11;
        public static int buttonId_gh = 3;
        public static int buttonId_ij = 1;
        public static int buttonId_kl = 8;
        public static int buttonId_AlgaeIntake = 4;
        public static int buttonId_AlgaeOutake = 2;

        // driver button ids
        public static int buttonId_Climb = 4;
        public static int buttonId_LeftFeederSt = 5;
        public static int buttonId_RightFeederSt = 6;
        public static int axisId_RightBranch = 3;
        public static int axisId_LeftBranch = 2;
        public static int buttonId_reorenting = 3;
        public static int buttonId_resetOdo = 8;
        public static int buttonId_processorAP = 2;

        // auto poses
        // get aprilTag pose on blue
        public static Pose2d cdATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(17)
                        .get()
                        .toPose2d();
        public static Pose2d efATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(22)
                        .get()
                        .toPose2d();
        public static Pose2d ghATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21)
                        .get()
                        .toPose2d();
        public static Pose2d processorATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(16)
                        .get().toPose2d();
        public static Pose2d abATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18)
                        .get()
                        .toPose2d();
        public static Pose2d ijATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(20)
                        .get()
                        .toPose2d();
        public static Pose2d klATPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(19)
                        .get()
                        .toPose2d();
        public static Pose2d rightFeederStationPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(12).get().toPose2d();
        public static Pose2d leftFeederStationPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(13).get().toPose2d();

        // offsets reef
        public static Transform2d tagToLeft = new Transform2d(0.475, -0.227, new Rotation2d(Math.PI));
        public static Transform2d tagToRight = new Transform2d(0.475, 0.1, new Rotation2d(Math.PI));

        // offset pocessor
        public static Transform2d tagToProcessor = new Transform2d(0.4,0.1, new Rotation2d(Math.PI));
        public static Transform2d closeFeederStation = new Transform2d(0.5, -0.1, new Rotation2d());
        public static Transform2d farFeederStation = new Transform2d(0.5, 0.4, new Rotation2d());

        // astar tranforms
        public static Transform2d astarReefPoseOffset = new Transform2d(-0.4, 0, new Rotation2d());
        public static Transform2d astarProcesserPoseOffset = new Transform2d(-1, 0, new Rotation2d());
        public static Transform2d astarFeederStPoseOffset = new Transform2d(0.425, 0, new Rotation2d());

        // reef poses
        public static Pose2d coralRightAB = abATPose.plus(tagToRight);
        public static Pose2d coralLeftAB = abATPose.plus(tagToLeft);
        public static Pose2d coralRightCD = cdATPose.plus(tagToRight);
        public static Pose2d coralLeftCD = cdATPose.plus(tagToLeft);
        public static Pose2d 
        coralRightEF = efATPose.plus(tagToRight);
        public static Pose2d coralLeftEF = efATPose.plus(tagToLeft);
        public static Pose2d coralRightGH = ghATPose.plus(tagToRight);
        public static Pose2d coralLeftGH = ghATPose.plus(tagToLeft);
        public static Pose2d coralRightIJ = ijATPose.plus(tagToRight);
        public static Pose2d coralLeftIJ = ijATPose.plus(tagToLeft);
        public static Pose2d coralRightKL = klATPose.plus(tagToRight);
        public static Pose2d coralLeftKL = klATPose.plus(tagToLeft);

        public static Pose2d processorAP = processorATPose.plus(tagToRight);
        public static Pose2d rightCloseFeederStationAP = rightFeederStationPose.plus(closeFeederStation);
        public static Pose2d rightFarFeederStationAP = rightFeederStationPose.plus(farFeederStation);
        public static Pose2d leftFarFeederStationAP = leftFeederStationPose.plus(farFeederStation);
        public static Pose2d leftCloseFeederStationAP = leftFeederStationPose.plus(closeFeederStation);

        public static Pose2d reefZoneBlue = new Pose2d(4.495, 4.019, new Rotation2d(Radians.convertFrom(0, Degrees)));
        public static Pose2d reefZoneRed = new Pose2d(13.091, 4.043, new Rotation2d(Radians.convertFrom(0, Degrees)));
        public static double minDistanceFromReefZoneMeter = 3;
        public static double maxATDist = 2.5;

        public static double coralScoreThold = 0.02;
        public static double otfToAPThold = 0.25;

        // elevator positions
        public static double elevatorPosStowed = 0;
        public static double elevatorPosL1 = 0;
        public static double elevatorPosL2 = 7.00;
        public static double elevatorPosL3 = 20;
        public static double elevatorPosL4 = 41.4;
        public static double elevatorPosA2 = 23.43;
        public static double elevatorPosA3 = 34.57;

        // coral sensor
        public static double sensorThold = 1;

        // auto start poses
        public static Pose2d autoMidStartPose = new Pose2d(7.229, 3.899, new Rotation2d(Math.PI));
        public static Pose2d autoProcessorStartPose = new Pose2d(7.229, 1.538, new Rotation2d(Math.PI));
        public static Pose2d autoFarStartPose = new Pose2d(7.229, 6.453, new Rotation2d(Math.PI));

        public static double getDistanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
                double x = pose2.getX() - pose1.getX();
                double y = pose2.getY() - pose1.getY();
                double distance = Math.sqrt(x * x + y * y);
                return distance;
        }
}
