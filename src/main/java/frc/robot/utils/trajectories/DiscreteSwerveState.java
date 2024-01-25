package frc.robot.utils.trajectories;

import frc.robot.subsystems.swerve.SwerveState;

public class DiscreteSwerveState extends SwerveState implements Comparable<Double>{
    public final double time;
    
    public DiscreteSwerveState(SwerveState state, double time){
        super(state);
        this.time = time;
    }

    @Override
    public int compareTo(Double arg0) {
        // TODO Auto-generated method stub
        return Double.compare(time, arg0);
    }

    
    
}