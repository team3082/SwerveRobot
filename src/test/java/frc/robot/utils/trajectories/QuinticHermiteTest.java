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
        try{
            QuinticHermite spline = new QuinticHermite(states, 1);
            SwerveState state = spline.get(0.5);
            assertEquals(6.563, state.x,0.001);
            assertEquals(3.438, state.y,0.001);
            assertEquals(0, state.theta,0.001);
            
            QuinticHermite spline2 = new QuinticHermite(states, 10);
            SwerveState state2 = spline2.get(5);
            assertEquals(state.x, state2.x, 0.0001);
            assertEquals(10.0, state.dx/state2.dx, 0.0001);
            assertArrayEquals(spline.endState().toArray(), states[3].toArray(), 0.0001);
        }catch(Exception e){
            e.printStackTrace();
            assertFalse(true);
        }
    }
}
