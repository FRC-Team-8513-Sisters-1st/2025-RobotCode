package frc.robot.Logic.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;

public class InitAutoCommand extends AutoCommand {

    Pose2d startingPose;
    
    public InitAutoCommand(Robot robot, Pose2d startingPose) {
        super(robot);
        this.startingPose = startingPose;
    }

    public void initializeAuto() {
        if (Robot.isSimulation()) {
            if (robot.onRedAlliance) {
                robot.drivebase.swerveDrive.resetOdometry(robot.drivebase.flipPoseToRed(startingPose));
            } 
            else {
                robot.drivebase.swerveDrive.resetOdometry(startingPose);
            }
        }
        isComplete = true;
    }

    @Override
    public void execute() {
        initializeAuto();
    }
}
