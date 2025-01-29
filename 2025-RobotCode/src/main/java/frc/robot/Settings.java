package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

public final class Settings {
  // drivebase settings
  public static double drivebaseMaxVelocityFPS = 17.1;

  // joystick deadband
  public static double joystickDeadband = 0.01;

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
    public static int algaeMotor1CANID = 11;
    public static int algaeMotor2CANID = 12;
    public static int algaeGroundMotor1CANID = 13;
    public static int algaeGroundMotor2CANID = 14;
    public static int climberMotor1CANID = 15;
    public static int climberMotor2CANID = 16;
    public static int elevatorMotor1CANID = 17;
    public static int elevatorMotor2CANID = 18;
    public static int coralMotor1CANID = 19;
    public static int coralMotor2CANID = 20;


    // controller 1 panel buttons
    public static int buttonId_CoralIntake = 1;
    public static int buttonId_CoralOutake = 2;
    public static int buttonId_Coral4 = 3;
    public static int buttonId_Coral3 = 4;
    public static int buttonId_Coral2 = 5;
    public static int buttonId_Coral1 = 6;
    public static int buttonId_Algae3 = 7;
    public static int buttonId_Algae2 = 8;
    public static int buttonId_AlgaeIntake = 9;
    public static int buttonId_AlgaeOutake = 10;

        // controller 2 panel buttons
    public static int buttonId_Close = 1;
    public static int buttonId_Far = 2;
    public static int buttonId_Drive = 3;
    public static int buttonId_ab = 4;
    public static int buttonId_cd = 5;
    public static int buttonId_ef = 6;
    public static int buttonId_gh = 7;
    public static int buttonId_ij = 8;
    public static int buttonId_kl = 9;

    // driver button ids
    public static int buttonId_Climb = 1;
    public static int buttonId_ClimbDown = 2;
    public static int buttonId_LeftFeederSt = 3;
    public static int buttonId_RightFeederSt = 4;
    public static int buttonId_RightBranch = 5;
    public static int buttonId_LeftBranch = 6;
    public static int buttonId_reorenting = 7;
    public static int buttonId_processor = 8;


    // auto poses
    public static Pose2d leftCoralStationFar = new Pose2d();
    public static Pose2d rightCoralStationFar = new Pose2d();
    public static Pose2d leftCoralStationClose = new Pose2d();
    public static Pose2d rightCoralStationClose = new Pose2d();
    public static Pose2d coralRightAB = new Pose2d();
    public static Pose2d coralLeftAB = new Pose2d();
    public static Pose2d coralRightCD = new Pose2d();
    public static Pose2d coralLeftCD = new Pose2d();
    public static Pose2d coralRightEF = new Pose2d();
    public static Pose2d coralLeftEF = new Pose2d();
    public static Pose2d coralRightGH = new Pose2d();
    public static Pose2d coralLeftGH = new Pose2d();
    public static Pose2d coralRightIJ = new Pose2d();
    public static Pose2d coralLeftIJ = new Pose2d();
    public static Pose2d coralRightKL = new Pose2d();
    public static Pose2d coralLeftKL = new Pose2d();
    public static Pose2d rightFarFeederStation = new Pose2d();
    public static Pose2d leftFarFeederStation = new Pose2d();
    public static Pose2d rightCloseFeederStation = new Pose2d();
    public static Pose2d leftCloseFeederStation = new Pose2d();
    public static Pose2d processor = new Pose2d();


    public static Pose2d reefZone = new Pose2d();
    public static double minDistanceFromReefZoneMeter = 1;

}
