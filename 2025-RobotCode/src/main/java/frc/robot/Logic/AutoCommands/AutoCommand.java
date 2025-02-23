package frc.robot.Logic.AutoCommands;

import frc.robot.Robot;

public class AutoCommand {
    
    protected Robot robot;
    protected boolean isComplete = false;

    public AutoCommand(Robot robot) {
        this.robot = robot;
    }

    public boolean isComplete() {
         return isComplete;
    }

    public void execute() {}
}
