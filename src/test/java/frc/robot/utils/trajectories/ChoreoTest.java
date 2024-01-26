package frc.robot.utils.trajectories;


import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;
import static org.junit.jupiter.api.Assertions.*;
import java.io.File;

import static frc.robot.Constants.METERSTOINCHES;

import frc.robot.utils.trajectories.*;

public class ChoreoTest {

    @BeforeAll
    public static void start(){
        ChoreoTrajectoryGenerator.init();
    }

    @Test
    public void ChoreoTrajectoryTest(){
        DiscreteTraj dt = ChoreoTrajectoryGenerator.generateTrajectory(new File("src/main/deploy/deploy/choreo/UTest.traj"));
        assertEquals(dt.length(), 3.3565504113888167, 0.0001);
        assertEquals(dt.get(1.7292593899659614).x, 6.423343099411647 * METERSTOINCHES, 0.0001);
    }

    @Test
    public void LongTest(){
        DiscreteTraj dt = ChoreoTrajectoryGenerator.generateTrajectory(new File("src/main/deploy/deploy/choreo/New Path.traj"));
        // assertEquals(ct.length(), 3.3565504113888167, 0.0001);
        // assertEquals(ct.get(1.7292593899659614).x, 6.423343099411647 * METERSTOINCHES, 0.0001);
    }
}
