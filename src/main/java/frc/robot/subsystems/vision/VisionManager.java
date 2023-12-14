package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.utilities.Vector2;

public class VisionManager {
    Camera[] cameras;

    public VisionManager() {
        cameras = new Camera[] {
            new Camera(Constants.CAMERA_1_POSITION, Constants.CAMERA_1_ROTATION, Constants.CAMERA_1_ID)
        };
    }

    // averages out camera values
    public Vector2 getPosition() {
        Vector2 position = new Vector2();
        return position;
    }

    // gets the rotation of the robot, from camera data
    public double getRotation() {
        double rotation = 0.0;
        return rotation;
    }

    // tells cameras to get latest position and rotation values
    public void update() {
        for (Camera camera : cameras) {
            camera.update();
        }
    }
}
