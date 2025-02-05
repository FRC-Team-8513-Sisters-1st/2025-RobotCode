package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.logic.AutoController;
import frc.robot.logic.Dashboard;
import frc.robot.logic.StateMachine;
import frc.robot.logic.TeleopController;
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
  public Vision vision;
  public Dashboard dashboard;

  //variables
  public boolean onRedAlliance;
  public boolean coralReady2Score = false;
  public boolean algaeReady2Score = false;

  // controllers
  Joystick controller1 = new Joystick(1);
  Joystick buttonPannel1 = new Joystick(2);
  Joystick buttonPannel2 = new Joystick(3);
  public static int yAxisLeft = 1;
  public static int yAxisRight = 1;

  public Robot() {
    updateAllianceFromDS();
    drivebase = new Drivebase(this);
    teleopController = new TeleopController(this);
    autoController = new AutoController(this);
    stateMachine = new StateMachine(this);
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
