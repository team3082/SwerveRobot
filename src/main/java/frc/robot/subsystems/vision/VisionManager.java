package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.utilities.Vector2;

public class VisionManager {

    // Array of position-tracking cameras.
    private static Camera[] cameras;

    // The offsets (positions) of the camera on the robot.
    // Positive Y is forward and Positive X is right when looking from the front.
    private static Vector2[] offsets;

    // How many cameras do we have?
    private static double cameraAmount;

    public VisionManager() {
        cameras = new Camera[] {
            new Camera(Constants.CAMERA_1_POSITION, Constants.CAMERA_1_ROTATION, Constants.CAMERA_1_ID)
        };

        for (Camera camera: cameras) {
            offsets[camera.cameraID] = camera.cameraPosition;
        }

        cameraAmount = 4;
    }

    /**
     * Returns an averaged robot position 
     * based on the outputs of the cameras.
     * @return Vector2 representing the robot's position on the field.
     */
    public Vector2 getPosition() {
        Vector2 position = new Vector2();
        return position;
    }

    /**
     * Returns the averaged rotation of the robot,
     * in radians, gathered from the outputs of the cameras.
     * @return Robot's rotation on the field in radians.
     */
    public double getRotation() {
        double rotation = 0.0;
        return rotation;
    }

    /**
     * Instructs the cameras to periodically 
     * update their outputs from NetworkTables.
     */
    public void update() {
        for (Camera camera : cameras) {
            camera.update();
        }
    }
}
