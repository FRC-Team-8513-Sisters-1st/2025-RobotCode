// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Logic.TeleopController;
import frc.robot.Subsystems.Drivebase;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public Drivebase drivebase;
  public TeleopController teleopController;
  public boolean onRedAlliance;

  public Robot() {
    drivebase = new Drivebase(this);
    teleopController = new TeleopController(this);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    //drivebase.swerveDrive.drive(new ChassisSpeeds(3.0, -2.0, Math.PI));
  }

  @Override
  public void teleopInit() {
    drivebase.swerveDrive.resetOdometry(new Pose2d(2,2, new Rotation2d()));
  }

  @Override
  public void teleopPeriodic() {
    teleopController.driveTele();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
