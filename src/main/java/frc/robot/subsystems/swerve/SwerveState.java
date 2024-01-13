package frc.robot.subsystems.swerve;

import frc.robot.utilities.Math.RMath;
import frc.robot.utilities.Vector2;

public class SwerveState {
    //position of the robot in inches
    public final double x,y,theta;
    //change in position of the robot in inches
    public final double dx, dy, dtheta;

    public SwerveState(Vector2 pos, double theta, Vector2 dpos, double dtheta){
        x = pos.x;
        y = pos.y;
        this.theta = theta;
        dx = dpos.x;
        dy = dpos.y;
        this.dtheta = dtheta;
    }

    public SwerveState(double x, double y, double theta, double dx, double dy, double dtheta){
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public SwerveState(double[] pos, double[] dpos){
        if(pos.length != 3 || dpos.length != 3){
            throw new IllegalArgumentException("Arrays must be of length 3");
        }
        x = pos[0];
        y = pos[1];
        theta = pos[2];
        dx = dpos[0];
        dy = dpos[1];
        dtheta = dpos[2];
    }

    /** Takes array of length 3 or length 6, if the array is length 3 it will assume that velocity is 0*/
    public SwerveState(double[] state){
        if(state.length == 6){
            x = state[0];
            y = state[1];
            theta = state[2];
            dx = state[3];
            dy = state[4];
            dtheta = state[5];
        }else if(state.length == 3){
            x = state[0];
            y = state[1];
            theta = state[2];
            dx = 0;
            dy = 0;
            dtheta = 0;
        }else{
            throw new IllegalArgumentException("Argument must be of length 3 or 6");
        }

    }

    //TODO might make a specific class for swerve errors to reduce ambiguity
    /**
     * Finds the error between this state a desired state
     * @param target the desired swervestate
     * @return an array containing the error between this SwerveState and the target state
     */
    public double[] getError(SwerveState target){
        double[] targetArr = target.toArray();
        double[] thisArr = this.toArray();
        double[] ret = new double[6];
        for(int i = 0; i < 6; i++){
            ret[i] = targetArr[i] - thisArr[i];
        }
        ret[2] = RMath.targetAngleAbsolute(thisArr[2], targetArr[2], Math.PI * 2) - thisArr[2];
        
        return ret;
    }

    /**returns a array representation of this swerveState*/
    public double[] toArray(){
        return new double[]{x, y, theta, dx, dy, dtheta};
    }

    public String toString(){
        return String.format("x: % .2f %ny: % .2f %ntheta: % .2f %ndx: % .2f %ndy: % .2f %ndtheta: % .2f", x,y,theta,dx,dy,dtheta);
    }
}
