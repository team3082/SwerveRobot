package frc.robot.utils.followers;


import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;
import frc.robot.utils.trajectories.SwerveTrajectory;

public class PIDFollower extends SwerveFollower{
    double kPpos, kIpos,kDpos;
    double kProt, kIrot, kDrot;
    double[] intAccum = new double[3];
    double[] maxIntAccum;
    private double timeScale = 1.0;

    public PIDFollower(SwerveTrajectory traj, double kppos, double kipos, double kdpos, double kprot, double kirot, double kdrot, double timeScale, double[] maxIntAccum){
        super(traj);
        this.kPpos = kppos;
        this.kIpos = kipos;
        this.kDpos = kdpos;
        this.kProt = kprot;
        this.kIrot = kirot;
        this.kDrot = kdrot;
        this.timeScale = timeScale;
        this.maxIntAccum = maxIntAccum;
    }

    public SwerveInstruction getInstruction(SwerveState currentState, double t){
        SwerveState desiredState = path.get(t * timeScale);
        double[] error = currentState.getError(desiredState);
        //updating int accumulator
        intAccum = updateAccumulator(intAccum, currentState.toArray(), maxIntAccum);
        

        Vector2 trans = new Vector2(error[0] * kPpos + error[3] * kDpos + intAccum[0] * kIpos, error[1] * kPpos + error[4] * kDpos + intAccum[1] * kIpos);
        double rot = kProt * error[2] + kDrot * error[5] + kIrot * intAccum[2];
        
        return new SwerveInstruction(rot, trans);
    }

    private double[] updateAccumulator(double[] accumulator, double[] currentState, double[] maxValues){
        double[] ret = new double[accumulator.length];
        for(int i = 0; i < accumulator.length; i++){
            ret[i] = RMath.clamp(accumulator[i] + currentState[i], -maxValues[i], maxValues[i]);
        }
        return ret;
    }
    
}