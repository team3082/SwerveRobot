package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class SwervePosition {

    private static Vector2 position;
    private static Vector2 absVelocity;
    private static Vector2 lastAbsVelocity;

    public static void init() {
        absVelocity     = new Vector2();
        lastAbsVelocity = new Vector2();
        position        = new Vector2();
    }

    public static void update() {

        // Derive our velocity 
        Vector2 vel = SwerveManager.getRobotDriveVelocity();

        // Telemetry.log(Severity.INFO, "drive: " + vel);

        // Rotate our velocity to be local to the field
        vel = vel.rotate(Pigeon.getRotationRad() - Math.PI / 2);

        // Flip the x component of our velocity if we're on the red alliance
        // I still don't know why, but we don't need to do this in simulation mode
        if (DriverStation.getAlliance() == Alliance.Red)
            vel.x *= -1;
        
        lastAbsVelocity = absVelocity; 
        absVelocity = vel;

        // Integrate our velocity to find our position
        position = position.add(absVelocity.add(lastAbsVelocity).mul(0.5 * RTime.deltaTime()));
    }

    //returns array of the robot's angle and distance in INCHES based of manual calculations
    public static double[] getPositionPolar() {
        
        Vector2 pos = getPosition();
        double distance = pos.mag();
        double angleRad = pos.atan2();

        return new double[] { angleRad, distance };
    }

    public static Vector2 getNetDisplacement() {
        return position;
    }

    public static Vector2 getPosition() {
        return position;
    }

    public static Vector2 getAbsVelocity() {
        return absVelocity;
    }

    /**
     * Recalibrates the SwervePosition based on a position on the field. Should only be used when vision is disabled,
     * otherwise it'll just be overwritten the next frame.
     * @param newPosition the new position to set the robot position to
     */
    public static void setPosition(Vector2 newPosition) {
        position = newPosition;
    }

}