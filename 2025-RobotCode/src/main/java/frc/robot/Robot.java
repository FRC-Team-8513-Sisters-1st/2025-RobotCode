package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Logic.AutoController;
import frc.robot.Logic.Dashboard;
import frc.robot.Logic.TeleopController;
import frc.robot.Logic.Vision;
import frc.robot.Subsystems.Algae;
import frc.robot.Subsystems.AlgaeGround;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Coral;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Elevator;

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
  public Vision vision;
  public Dashboard dashboard;

  //variables
  public boolean onRedAlliance;

  public Robot() {
    updateAllianceFromDS();
    drivebase = new Drivebase(this);
    teleopController = new TeleopController(this);
    autoController = new AutoController(this);
    vision = new Vision(this);
    elevator = new Elevator(this);
    coral = new Coral(this);
    climber = new Climber(this);
    algae = new Algae(this);
    dashboard = new Dashboard(this);

  }

  @Override
  public void robotPeriodic() {
    dashboard.updateDashboard();
    if (Robot.isReal() && Settings.useVision) {
      vision.updatePhotonVision();
    } else {
      if(Robot.isSimulation()){
        drivebase.matchSimulatedOdomToPose();
      }
    }
  }

  @Override
  public void autonomousInit() {
    updateAllianceFromDS();
    autoController.autoInit();
  }

  @Override
  public void autonomousPeriodic() {
    autoController.autoPeriodic();
  }

  @Override
  public void teleopInit() {
    updateAllianceFromDS();
    teleopController.initTele();
  }

  @Override
  public void teleopPeriodic() {
    teleopController.driveTele();
  }

  @Override
  public void disabledInit() {
    vision.visionMaxATDist = Settings.maxATDistDisabeled;
  }

  @Override
  public void disabledPeriodic() {
    autoController.autoDis();
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
