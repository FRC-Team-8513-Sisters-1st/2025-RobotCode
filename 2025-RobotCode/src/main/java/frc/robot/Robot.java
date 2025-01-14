package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.logic.TeleopController;
import frc.robot.subsystems.Drivebase;

public class Robot extends TimedRobot {

  //subsystems
  public Drivebase drivebase;

  //logic
  public TeleopController teleopController;

  //variables
  public boolean onRedAlliance;

  double rV = 3;

  public Robot() {
    updateAllianceFromDS();
    drivebase = new Drivebase(this);
    teleopController = new TeleopController(this);
    
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    updateAllianceFromDS();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    updateAllianceFromDS();
    drivebase.swerveDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d()));
  }

  @Override
  public void teleopPeriodic() {
    teleopController.driveTele();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public void updateAllianceFromDS() {
    // checks driverstation for alliance and sets vaiable
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
      onRedAlliance = true;
    }else{
      onRedAlliance = false;
    }
  }
}
