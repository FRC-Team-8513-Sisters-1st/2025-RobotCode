package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class Settings {
  // drivebase settings
  public static double drivebaseMaxVelocityFPS = 17.1;

  // joystick deadband
  public static double joystickDeadband = 0.01;

   // driver axis 
   public static int leftRightAxis = 0;
   public static int forwardBackwardsAxis = 1;
   public static int rotAxis = 4; // 2 at home, 4 on xbox

   // driver joystick settings
   public static int driverJoystickPort = 0;

   // pid settings
    public static PIDController xController = new PIDController(33, 0, 1);
    public static PIDController yController = new PIDController(33, 0, 1);
    public static PIDController rController = new PIDController(1, 0, 0);
}
