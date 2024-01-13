package frc.robot.utils.followers;

import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.utils.trajectories.SwerveTrajectory;

public abstract class SwerveFollower {
    public final SwerveTrajectory path;

    public SwerveFollower(SwerveTrajectory tj){
        path = tj;
    }

    public abstract SwerveInstruction getInstruction(SwerveState currentState, double t);
}
