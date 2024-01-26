package frc.robot.utils.trajectories;

import java.util.Collections;
import java.util.List;

import frc.robot.subsystems.swerve.SwerveState;

public abstract class DiscreteTraj implements SwerveTrajectory{
    protected List<DiscreteSwerveState> path;

    @Override
    public DiscreteSwerveState endState(){
        return path.get(path.size() - 1);
    }

    @Override
    public SwerveState startState(){
        return path.get(0);
    }

    @Override
    public double length(){
        return endState().time;
    }

    public SwerveState get(double t){
        int posa = Collections.binarySearch(path, t);
        if(posa == path.size()){
            return endState();
        }
        SwerveState a = path.get(posa);
        SwerveState b = path.get(posa+1);
        double deltaT = t - path.get(posa).time;
        return a.interpolate(b, deltaT);
    }

    

}
