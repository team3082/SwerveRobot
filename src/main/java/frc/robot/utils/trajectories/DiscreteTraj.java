package frc.robot.utils.trajectories;

import java.util.Collections;
import java.util.List;

import frc.robot.subsystems.swerve.SwerveState;

public abstract class DiscreteTraj implements SwerveTrajectory{
    private final List<DiscreteSwerveState> path = null;

    @Override
    public SwerveState endState(){
        return path.get(path.size() - 1);
    }

    @Override
    public SwerveState startState(){
        return path.get(0);
    }

    public SwerveState get(double t){
        int posa = Collections.binarySearch(path, t);
        if(posa == path.size()){
            return endState();
        }
        SwerveState a = path.get(posa);
        SwerveState b = path.get(posa+1);
        double deltaT = path.get(posa+1).time - path.get(posa).time;
        return a.interpolate(b, deltaT);
    }

    

}
