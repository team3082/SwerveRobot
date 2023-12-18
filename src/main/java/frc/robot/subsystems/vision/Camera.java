package frc.robot.subsystems.vision;

import frc.robot.subsystems.NetworkTables;
import frc.robot.utilities.Vector2;

public class Camera {

    /** The ID of the camera. */
    public int cameraID;

    /** The position of the camera on the robot. */
    public Vector2 cameraPosition;

    /** The angle of the camera on the robot. */
    public double cameraAngle;

    /** The raw pose data gathered from NetworkTables. */
    public Vector2 cameraPositionData;

    /** The raw rotation data gathered from NetworkTables. */
    public double cameraRotationData;

    public boolean targetDetected;
    public int targetID;

    public Camera(Vector2 position, double angle, int id) {
        this.cameraPosition = position;
        this.cameraAngle = angle;
        this.cameraID = id;
    }

    /**
     * Periodically populate pose and rot from NetworkTables.
     */
    public void update() {
        cameraPositionData = new Vector2(NetworkTables.getX(), NetworkTables.getY());
        cameraRotationData = NetworkTables.getRot();
        targetDetected = NetworkTables.targetDetected.getBoolean(false);
        targetID = (int) NetworkTables.PLACEHOLDER_TARGETID.getDouble(0);
    }

    /**
     * Fetch the raw position from NetworkTables.
     * @return Raw position gathered from AprilTag data
     */
    public Vector2 getPosition() {
        return this.cameraPositionData;
    }

    /**
     * Fetch the raw rotation from NetworkTables.
     * @return Raw rotation gathered from AprilTag data
     */
    public double getRotation() {
        return this.cameraRotationData;
    }
}
