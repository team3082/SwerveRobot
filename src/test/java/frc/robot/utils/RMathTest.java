package frc.robot.utils;

import org.junit.jupiter.api.Test;

import frc.robot.utils.RMath;

import static org.junit.jupiter.api.Assertions.*;

public class RMathTest {
    @Test
    public void interpolateTest(){
        double a = 1;
        double b = 2;
        double t = 0.7;
        assertEquals(1.7, RMath.interpolate(a, b, t));
    }
}
