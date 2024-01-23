package frc.robot.utils.trajectories;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.swerve.SwerveState;

import static org.junit.jupiter.api.Assertions.*;

public class QuinticHermiteTest {
    
    @Test
    public void testSpline(){
        SwerveState[] states = new SwerveState[]{
            new SwerveState(new double[]{0,0,0}),
            new SwerveState(new double[]{5,0,0}),
            new SwerveState(new double[]{10,5,0}),
            new SwerveState(new double[]{10,10,0})
        };
        QuinticHermite spline = new QuinticHermite(states, 1);
        try{
            SwerveState state = spline.get(0.5);
            assertEquals(6.563, state.x,0.001);
            assertEquals(3.438, state.y,0.001);
            assertEquals(0, state.theta,0.001);
        }catch(Exception e){
            e.printStackTrace();
            assertFalse(true);
        }
    }
}
