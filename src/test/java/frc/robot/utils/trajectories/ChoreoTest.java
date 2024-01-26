import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import java.io.File;

import static frc.robot.Constants.METERSTOINCHES;

import frc.robot.utils.trajectories.ChoreoTrajectory;

public class ChoreoTest {
    @Test
    public void ChoreoTrajectoryTest(){
        ChoreoTrajectory ct = new ChoreoTrajectory(new File("src/main/deploy/deploy/choreo/UTest.traj"));
        assertEquals(ct.length(), 3.3565504113888167, 0.0001);
        assertEquals(ct.get(1.7292593899659614).x, 6.423343099411647 * METERSTOINCHES, 0.0001);
    }

}
