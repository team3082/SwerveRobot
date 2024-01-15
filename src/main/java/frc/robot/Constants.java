package frc.robot;

import frc.robot.utils.Vector2;

public class Constants {
    // Vision
    public static final int CAMERA_1_ID = 1; // ID
    public static final Vector2 CAMERA_1_POSITION = new Vector2(0, 0); // (x, y) position in relation to center of robot
    public static final double CAMERA_1_ROTATION = 0.0; // radians (0 is right)
    public static final int CAMERA_2_ID = 2; // ID
    public static final Vector2 CAMERA_2_POSITION = new Vector2(0, 0); // (x, y) position in relation to center of robot
    public static final double CAMERA_2_ROTATION = 0.0; // radians (0 is right)

    public static final double firstX   = 115.61;
    public static final double secondX  = 49.61;
    public static final double thirdX   = -16.61;
    public static final double gridY    = -285.16;
    public static final double loadingX = -107.94;
    public static final double loadingY = -311.35;

    public static final double wheelDiameter = 4;
    public static final double freeSpeedRPM = 5676;

    // Inches per second
    public static final double driveVelFactor = ((wheelDiameter * Math.PI) / 6.12) / 60;

    // Radians per second
    public static final double turnVelFactor = (2 * Math.PI) / 60.0;
}
