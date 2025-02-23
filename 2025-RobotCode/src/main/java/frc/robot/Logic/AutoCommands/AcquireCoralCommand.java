package frc.robot.Logic.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.CoralIntakeStates;
import frc.robot.Logic.Enums.ElevatorStates;

public class AcquireCoralCommand extends AutoCommand {

    Pose2d coralStation;
    double timeStepStarted = 0;
    boolean pathInit = false;

    public AcquireCoralCommand(Robot robot, Pose2d coralStation) {
        super(robot);
        this.coralStation = coralStation;
    }
    
    public void acquireCoral() {
        if(!pathInit){
            robot.drivebase.initAstarAndAP(coralStation.transformBy(Settings.astarFeederStPoseOffset), coralStation);
            pathInit = true;
        }
        robot.coral.state = CoralIntakeStates.outake;
        robot.coral.setMotorPower();
        if (timeStepStarted == 0) {
            timeStepStarted = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
            robot.elevator.state = ElevatorStates.L1;
        }
        robot.elevator.setMotorPower();
        if (robot.drivebase.fromOTFSwitchToAP()) {
            isComplete = true;
        }
    }

    @Override
    public void execute() {
        acquireCoral();
    }    
}
