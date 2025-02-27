 package frc.robot.Logic;

public final class Enums {
    public enum ClimberStates {
        stowed, armOut, climbing
    }
    public enum AlgaeGroundStates {
        intake, holdingAlgae, outake, stowed
    }
    public enum AlgaeIntakeStates {
        intake, holdPosition, outake, stationary
    }
    public enum CoralIntakeStates {
        intake, stationary, outake
    }
    public enum ElevatorStates {
        L1, L2, L3, L4,
        stowed, scoreProcessor, L3a, L2a 
    }
    public enum DrivebaseStates {
        locked, stowedLimits, L1Limits, L2Limits,
        L3Limits, L4Limits 
    }
    public enum RobotStates {
        driving, preClimb, climbDeep, algaeIntakeL3, algaeIntakeL2,
        algaeIntakeGround, algaeScoreProcessor, coralScore1, coralScore2, coralScore3,
        coralScore4, coralIntakeFeederSt, coral1, coral2, coral3, coral4 

    }
    public enum SideOfReef {
        AB, CD, EF, GH, IJ, KL
    }
    public enum FeederStation {
        Close, Far
    }
    public enum AutoRoutines {
        DoNothing, 
        mid_GH3R, 
        processor_EF2L, 
        processor_EF2L_RFS_AB2L_RFS_AB2R, 
        far_IJ2L_LFS_KL4R_LFS_KL4L,
        mid_EF2R_RFS_CD4R_RFS_CD4L,
        customAutoAnyLength,
        betWithBusler,
        hpPractice
        // make auto that lines up to score processor then score during tele
    }
}
