package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.Pigeon;
import frc.robot.utilities.Vector2;

public class VisionManager {

    // Array of position-tracking cameras.
    private static Camera[] cameras;

    // The offsets (positions) of the camera on the robot.
    // Positive Y is forward and Positive X is right when looking from the front.
    private static Vector2[] offsets;

    // The final, modified positions from the cameras
    private static Vector2[] finalPos;

    // Positions of the apriltags, in inches.
    private static final Vector2[] aprilTags = {
        new Vector2(Constants.firstX, Constants.gridY),
        new Vector2(Constants.secondX, Constants.gridY),
        new Vector2(Constants.thirdX, Constants.gridY),
        new Vector2(Constants.loadingX, Constants.loadingY)
    };

    // How many cameras do we have?
    private static final double cameraAmount = 4;

    public static void init() {
        cameras = new Camera[] {
            new Camera(Constants.CAMERA_1_POSITION, Constants.CAMERA_1_ROTATION, Constants.CAMERA_1_ID)
        };

        for (Camera camera: cameras) {
            offsets[camera.cameraID] = camera.cameraPosition;
        }
    }

    /**
     * Returns an averaged robot position 
     * based on the outputs of the cameras.
     * @return Vector2 representing the robot's position on the field.
     */
    public static Vector2 getPosition() throws Exception {

        Vector2 position = new Vector2();
        int targets = 0;

        for (Camera camera : cameras) {

            // Throw away any frames without a target
            if (camera.targetDetected == false)
                continue;

            if (camera.targetID > 8 || camera.targetID < 1)
                continue;

            // TODO:
            // Filter through the (many) tags we will find
            // to track the one with the lowest ambiguity.
            // For now, this assumes each camera will see one tag.

            // offset is the vector from the center of the robot to the apriltag
            Vector2 rawPos = camera.getPosition();
            Vector2 offset = new Vector2(-rawPos.y, rawPos.x);
            // Convert to inches
            offset = offset.mul(39.3701);
            offset.y *= Math.cos(Math.toRadians(17.5));
            // Point vector axes forward relative to robot.
            offset = offset.rotate(Math.PI / 2 - camera.cameraAngle);
            // Compensate for the camera's position on the robot.
            offset = offset.add(offsets[camera.cameraID]);
            // Make vector in field space instead of robot space.
            offset = toFieldSpace(offset, Pigeon.getRotationRad(), camera.targetID);

            finalPos[camera.cameraID] = offset;
            position = position.add(offset);
            targets++;
        }

        if (targets > 0)
           return position.div(cameraAmount);

        throw new Exception("No feasible targets! getPosition() ignored.");
    }

    /**
     * Returns the averaged rotation of the robot,
     * in radians, gathered from the outputs of the cameras.
     * @return Robot's rotation on the field in radians.
     */
    public static double getRotation() {
        double rotation = 0.0;
        return rotation;
    }

    /**
     * Instructs the cameras to periodically 
     * update their outputs from NetworkTables.
     */
    public static void update() {
        for (Camera camera : cameras) {
            camera.update();
        }
    }

    /**
     * Convert from a vector from center of robot to tag that is relative to robot rotation to
     * a vector from center of field to the robot relative to field rotation
     * @param offset the vector from the center of the robot to the tag, relative to the robot rotation
     * @param pigeonAngle the yaw of the pigeon in radians
     * @param tagID the ID of the AprilTag detected
     * @return a vector from the center of the field to the robot, relative to the field rotation
     */
    private static Vector2 toFieldSpace(Vector2 offset, double pigeonAngle, int tagID) {
        offset = offset.mul(-1);
        Vector2 tagRelOffset = offset.rotate(pigeonAngle - Math.PI / 2);

        // We want to flip the X of the offset from the tag, but not the position of the tag itself.
        if (DriverStation.getAlliance() == Alliance.Red)
                tagRelOffset.x *= -1;
        
        Vector2 absolutePos = getTagPos(tagID).add(tagRelOffset);

        return absolutePos;
    }

    public static boolean isTagFriendly(int tagID){
        switch (DriverStation.getAlliance()){
            case Red:
                return tagID < 5;
            case Blue:
                return tagID > 4;
            default:
                System.out.println("SOMETHING IS BAD");;
                return false;
        }
    }

    public static Vector2 getTagPos(int tagID){
        int index = tagID<5 ? tagID-1 : 8-tagID;
        
        // LEAVE IT LIKE THIS SO WE DON'T FLIP APRIL TAG POSITIONS
        Vector2 v = new Vector2(aprilTags[index].x, aprilTags[index].y);
        
        if(!isTagFriendly(tagID))
            v.y *= -1; // If it's enemy make tag y positive
        return v;
    }
}
