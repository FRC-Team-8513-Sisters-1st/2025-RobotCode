package frc.robot;

import java.util.Optional;

import com.reduxrobotics.canand.CanandDeviceDetails.Enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.logic.Auto;
import frc.robot.logic.AutoController;
import frc.robot.logic.StateMachine;
import frc.robot.logic.TeleopController;
import frc.robot.logic.Variables;
import frc.robot.logic.Vision;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.AlgaeGround;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {

  //subsystems
  public Drivebase drivebase;
  public Algae algae;
  public AlgaeGround algaeGround;
  public Climber climber;
  public Coral coral;
  public Elevator elevator;


  //logic
  public TeleopController teleopController;
  public AutoController autoController;
  public StateMachine stateMachine;
  public Enums enums;
  public Auto auto;
  public Variables variables;
  public Vision vision;

  //variables
  public boolean onRedAlliance;

  double rV = 3;

  public Robot() {
    updateAllianceFromDS();
    drivebase = new Drivebase(this);
    teleopController = new TeleopController(this);
    autoController = new AutoController(this);
    
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousInit() {
    autoController.autoInit();
    updateAllianceFromDS();
    drivebase.initPath("circle while turning");
  }

  @Override
  public void autonomousPeriodic() {
    autoController.autoPeriodic();
    if(drivebase.followLoadedPath()) {
      drivebase.swerveDrive.lockPose();
    }
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
