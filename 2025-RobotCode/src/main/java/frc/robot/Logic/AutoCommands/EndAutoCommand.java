package frc.robot.Logic.AutoCommands;

import frc.robot.Robot;

public class EndAutoCommand extends AutoCommand {
    
    public EndAutoCommand(Robot robot) {
        super(robot);
    }

    public void endAuto() {
        robot.elevator.setMotorPower();
        robot.coral.setMotorPower();
        robot.drivebase.swerveDrive.lockPose();
    }

    @Override
    public void execute() {
        endAuto();
    }
}
