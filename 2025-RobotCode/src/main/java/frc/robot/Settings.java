package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

        // pid path follow settings
        public static PIDController xController = new PIDController(5, 0, 0.01);
        public static PIDController yController = new PIDController(5, 0, 0.01);
        public static PIDController rController = new PIDController(0.1, 0, 0);

        // pid settings
        public static PIDController xControllerAP = new PIDController(0.1, 0.005, 0.01);
        public static PIDController yControllerAP = new PIDController(0.1, 0.005, 0.01);
        public static PIDController rControllerAP = new PIDController(0.1, 0, 0);

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
        public static int buttonId_Close = 7;
        public static int buttonId_Far = 6;
        public static int buttonId_Drive = 5;
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

        // auto poses
        // create offsets for the elevator and the distance from the reef
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
        public static Pose2d coralRightAB = new Pose2d(3.081, 3.851, new Rotation2d(Radians.convertFrom(0, Degrees)));
        public static Pose2d coralLeftAB = new Pose2d(3.069, 4.22, new Rotation2d(Radians.convertFrom(0, Degrees)));
        public static Pose2d coralRightCD = new Pose2d(3.88, 2.76, new Rotation2d(Radians.convertFrom(60, Degrees)));
        public static Pose2d coralLeftCD = new Pose2d(3.66, 2.96, new Rotation2d(Radians.convertFrom(60, Degrees)));
        public static Pose2d coralRightEF = new Pose2d(5.287, 2.904, new Rotation2d(Radians.convertFrom(120, Degrees)));
        public static Pose2d coralLeftEF = new Pose2d(5.059, 2.748, new Rotation2d(Radians.convertFrom(120, Degrees)));
        public static Pose2d coralRightGH = new Pose2d(5.874, 4.175, new Rotation2d(Radians.convertFrom(180, Degrees)));
        public static Pose2d coralLeftGH = new Pose2d(5.862, 3.863, new Rotation2d(Radians.convertFrom(180, Degrees)));
        public static Pose2d coralRightIJ = new Pose2d(5.023, 5.314,
                        new Rotation2d(Radians.convertFrom(-120, Degrees)));
        public static Pose2d coralLeftIJ = new Pose2d(5.35, 5.08, new Rotation2d(Radians.convertFrom(-120, Degrees)));
        public static Pose2d coralRightKL = new Pose2d(3.66, 5.10, new Rotation2d(Radians.convertFrom(-60, Degrees)));
        public static Pose2d coralLeftKL = new Pose2d(4, 5.29, new Rotation2d(Radians.convertFrom(-60, Degrees)));
        public static Pose2d rightFarFeederStation = new Pose2d(1.678, 0.746,
                        new Rotation2d(Radians.convertFrom(-130, Degrees)));
        public static Pose2d leftFarFeederStation = new Pose2d(1.690, 7.280,
                        new Rotation2d(Radians.convertFrom(130, Degrees)));
        public static Pose2d rightCloseFeederStation = new Pose2d(1.24, 0.95,
                        new Rotation2d(Radians.convertFrom(50, Degrees)));
        public static Pose2d leftCloseFeederStation = new Pose2d(0.887, 6.740,
                        new Rotation2d(Radians.convertFrom(130, Degrees)));
        public static Pose2d processor = new Pose2d(6.006, 0.603, new Rotation2d(Radians.convertFrom(-90, Degrees)));

        public static Pose2d reefZone = new Pose2d(4.495, 4.019, new Rotation2d(Radians.convertFrom(0, Degrees)));
        public static double minDistanceFromReefZoneMeter = 2;
        public static double maxATDist = 2.5;

        public static double coralScoreThold = 0.0254;

        // elevator positions
        public static double elevatorPosStowed = 0;
        public static double elevatorPosL1 = 0;
        public static double elevatorPosL2 = 7.00;
        public static double elevatorPosL3 = 20.84;
        public static double elevatorPosL4 = 41.4;
        public static double elevatorPosA2 = 10;
        public static double elevatorPosA3 = 20;

        public static double getDistanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
                double x = pose2.getX() - pose1.getX();
                double y = pose2.getY() - pose1.getY();
                double distance = Math.sqrt(x) + Math.sqrt(y);
                return distance;
        }
}
