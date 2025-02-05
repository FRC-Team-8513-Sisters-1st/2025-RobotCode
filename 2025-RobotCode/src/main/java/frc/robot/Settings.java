package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Settings {
  // drivebase settings
  public static double drivebaseMaxVelocityFPS = 17.1;

  //vision settings
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
    public static PIDController xController = new PIDController(33, 0, 1);
    public static PIDController yController = new PIDController(33, 0, 1);
    public static PIDController rController = new PIDController(1, 0, 0);

    // CANids
    public static int algaeMotor1CANID = 62;
    public static int algaeGroundMotor1CANID = 13;
    public static int algaeGroundMotor2CANID = 14;
    public static int climberMotor1CANID = 15;
    public static int climberMotor2CANID = 16;
    public static int elevatorMotor1CANID = 57; // right
    public static int elevatorMotor2CANID = 59; // left
    public static int coralMotor1CANID = 19;
    public static int coralMotor2CANID = 20;


    // controller 1 panel buttons
    public static int buttonId_CoralIntake = 8;
    public static int buttonId_CoralOutake = 9;
    public static int buttonId_Coral4 = 4;
    public static int buttonId_Coral3 = 3;
    public static int buttonId_Coral2 = 2;
    public static int buttonId_Coral1 = 1;
    public static int buttonId_Algae3 = 7;
    public static int buttonId_Algae2 = 5;
    public static int buttonId_AlgaeIntake = 10;
    public static int buttonId_AlgaeOutake = 11;
    public static int buttonId_processor = 6;

    // controller 2 panel buttons
    public static int buttonId_Close = 4;
    public static int buttonId_Far = 7;
    public static int buttonId_Drive = 1;
    public static int buttonId_ab = 6;
    public static int buttonId_cd = 9;
    public static int buttonId_ef = 8;
    public static int buttonId_gh = 5;
    public static int buttonId_ij = 3;
    public static int buttonId_kl = 2;

    // driver button ids
    public static int buttonId_Climb = 4;
    public static int buttonId_LeftFeederSt = 5;
    public static int buttonId_RightFeederSt = 6;
    public static int axisId_RightBranch = 3;
    public static int axisId_LeftBranch = 2;
    public static int buttonId_reorenting = 3;

    // auto poses
    // create offsets for the elevator and the distance from the reef
    public static Pose2d coralRightAB = new Pose2d(3.081, 3.851, new Rotation2d(Radians.convertFrom(0, Degrees)));
    public static Pose2d coralLeftAB = new Pose2d(3.069, 4.187, new Rotation2d(Radians.convertFrom(0, Degrees)));
    public static Pose2d coralRightCD = new Pose2d(3.944, 2.748, new Rotation2d(Radians.convertFrom(60, Degrees)));
    public static Pose2d coralLeftCD = new Pose2d(3.644, 2.916, new Rotation2d(Radians.convertFrom(60, Degrees)));
    public static Pose2d coralRightEF = new Pose2d(5.287, 2.904, new Rotation2d(Radians.convertFrom(120, Degrees)));
    public static Pose2d coralLeftEF = new Pose2d(5.059, 2.748, new Rotation2d(Radians.convertFrom(120, Degrees)));
    public static Pose2d coralRightGH = new Pose2d(5.874, 4.175, new Rotation2d(Radians.convertFrom(180, Degrees)));
    public static Pose2d coralLeftGH = new Pose2d(5.862, 3.863, new Rotation2d(Radians.convertFrom(180, Degrees)));
    public static Pose2d coralRightIJ = new Pose2d(5.023, 5.314, new Rotation2d(Radians.convertFrom(-120, Degrees)));
    public static Pose2d coralLeftIJ = new Pose2d(5.311, 5.134, new Rotation2d(Radians.convertFrom(-120, Degrees)));
    public static Pose2d coralRightKL = new Pose2d(3.644, 5.098, new Rotation2d(Radians.convertFrom(-60, Degrees)));
    public static Pose2d coralLeftKL = new Pose2d(3.956, 5.266, new Rotation2d(Radians.convertFrom(-60, Degrees)));
    public static Pose2d rightFarFeederStation = new Pose2d(1.678, 0.746, new Rotation2d(Radians.convertFrom(-130, Degrees)));
    public static Pose2d leftFarFeederStation = new Pose2d(1.690, 7.280, new Rotation2d(Radians.convertFrom(130, Degrees)));
    public static Pose2d rightCloseFeederStation = new Pose2d(0.851, 1.370, new Rotation2d(Radians.convertFrom(-130, Degrees)));
    public static Pose2d leftCloseFeederStation = new Pose2d(0.887, 6.740, new Rotation2d(Radians.convertFrom(130, Degrees)));
    public static Pose2d processor = new Pose2d(6.006, 0.603, new Rotation2d(Radians.convertFrom(-90, Degrees)));

    public static Pose2d reefZone = new Pose2d(4.495, 4.019, new Rotation2d(Radians.convertFrom(0, Degrees)));
    public static double minDistanceFromReefZoneMeter = 1;
    
    public static double coralScoreThold = 0.0254;

    public static double getDistanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
        double x = pose2.getX() - pose1.getX();
        double y = pose2.getY() - pose1.getY();
        double distance = Math.sqrt(x) + Math.sqrt(y);
        return distance;
    }
}
