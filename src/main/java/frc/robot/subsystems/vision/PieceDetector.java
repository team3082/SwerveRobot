package frc.robot.subsystems.vision;

import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.Pigeon;
import frc.robot.utils.Vector2;

public class PieceDetector {
    
    public static double distance = 0;

    // Should be placed in the VisionManager.update() function
    public static void update() {
        distance = NetworkTables.getVel();
    }

    public static double getRawDist() {
        return distance;
    }

    public static Vector2 getFieldPos() {
        return Vector2.fromPolar(Pigeon.getRotationRad(), getRawDist());
    }
}
