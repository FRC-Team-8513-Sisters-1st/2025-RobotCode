// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
  Joystick joystick = new Joystick(0);
  private final SparkMax m_frontleftMotor = new SparkMax(Settings.kFrontLeftMotorPort);
  private final SparkMax m_frontrightMotor = new SparkMax(Settings.kFrontRightMotorPort);
  private final SparkMax m_backleftMotor = new SparkMax(Settings.kBackLeftMotorPort);
  private final SparkMax m_backrightMotor = new SparkMax(Settings.kBackRightMotorPort);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_frontleftMotor::set, m_frontrightMotor::set, m_backleftMotor::set, m_backrightMotor::set);
  private final GenericHID m_stick = new GenericHID(Constants.kJoystickPort);
  m_robotDrive.arcadeDrive(-m_stick.getRawAxis(0), m_stick.getRawAxis(1));

  public Drivebase drivebase;
  public boolean onRedAlliance;
  public Robot() {
    drivebase = new Drivebase(this);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    drivebase.swerveDrive.drive(new ChassisSpeeds(3.0, -2.0, Math.PI));
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drivebase.swerveDrive.drive(new ChassisSpeeds(vx, vy, vt));
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
