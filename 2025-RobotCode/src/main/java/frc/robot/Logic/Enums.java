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
    public enum SideOfReef {
        AB, CD, EF, GH, IJ, KL
    }
    public enum FeederStation {
        Close, Far
    }
    public enum AutoRoutines {
        DoNothing, 
        processor_EF2L_RFS_AB2L_RFS_AB2R, 
        processor_EF2L_RFS_CD4L_RFS_CD4R,
        processor_EF4L_RFS_AB4L_RFS_AB4R,
        processor_EF4L_RFS_CD4L_RFS_CD4R,
        processor_CD4R_RFS_CD4L_RFS_AB4R, 
        far_IJ2L_LFS_KL4R_LFS_KL4L,
        far_IJ2L_LFS_KL2R_LFS_KL2L,
        far_KL4R_LFS_AB4L_LFS_AB4R,
        far_IJ4R_LFS_KL4L_LFS_KL4R,
        mid_GH2R_RFS_CD4R_RFS_CD4L,
        mid_GH2R_LFS_KL4L_LFS_KL4R,
        mid_EF2R_RFS_CD4R_RFS_CD4L,
        mid_IJ2L_LFS_KL4L_LFS_KL4R,
        mid_GH2R,
        mid_GH4R,        
        customAutoAnyLength,
        betWithBusler,
        hpPractice
        // make auto that lines up to score processor then score during tele
    }
}
