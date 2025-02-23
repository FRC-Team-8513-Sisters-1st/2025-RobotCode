package frc.robot.Logic.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.CoralIntakeStates;
import frc.robot.Logic.Enums.ElevatorStates;

public class ScoreOnReefCommand extends AutoCommand {

    private Pose2d goalBranch;
    private ElevatorStates elevatorState;
    private boolean reachedDestination = false;
    private boolean pathInit = false;
    double timeStepStarted = 0;

    public ScoreOnReefCommand(Robot robot, Pose2d goalBranch, ElevatorStates elevatorState) {
        super(robot);
        this.goalBranch = goalBranch;
        this.elevatorState = elevatorState;
    }

    public void score(){
        if(!pathInit){
            robot.drivebase.initAstarAndAP(goalBranch.transformBy(Settings.astarReefPoseOffset), goalBranch);
            pathInit = true;
        }
        robot.coral.state = CoralIntakeStates.stationary;
        robot.elevator.state = elevatorState;
        if (!reachedDestination && robot.drivebase.fromOTFSwitchToAP()) {
            reachedDestination = true;
            robot.coral.state = CoralIntakeStates.outake;
            timeStepStarted = Timer.getFPGATimestamp();
        }
        robot.elevator.setMotorPower();
        robot.coral.setMotorPower();
        if (reachedDestination) {
            robot.drivebase.swerveDrive.lockPose();
            if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                isComplete = true;
            }
        }
    
    }

    @Override
    public void execute() {
        score();
    }
    
}
