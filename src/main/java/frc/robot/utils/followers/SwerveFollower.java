package frc.robot.utils.followers;

import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.utils.trajectories.SwerveTrajectory;

public abstract class SwerveFollower {
    public SwerveTrajectory path;

    public void setTrajectory(SwerveTrajectory traj){
        this.path = traj;
    }
    
    public abstract SwerveInstruction getInstruction(SwerveState currentState, double t);
}
