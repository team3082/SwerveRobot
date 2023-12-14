package frc.robot.subsystems.vision;

import frc.robot.utilities.Vector2;

public class Camera {
    int cameraID;
    Vector2 cameraPosition;
    double cameraAngle;

    Vector2 cameraPositionData;
    double cameraRotationData;

    public Camera(Vector2 position, double angle, int id) {
        this.cameraPosition = position;
        this.cameraAngle = angle;
        this.cameraID = id;
    }

    public void update() {

    }

    public Vector2 getPosition() {
        return this.cameraPositionData;
    }

    public double getRotation() {
        return this.cameraRotationData;
    }
}
