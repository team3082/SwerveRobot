package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.vision.VisionManager;
import frc.robot.utilities.Vector2;
import frc.robot.utilities.Math.RTime;

public class SwervePosition {

    // Smoothly correct field position based on vision output. VISION_CORRECTION_FACTOR should range from 0.0 to
    // 1.0, representing the speed at which we blend from the odometry output to the output of the vision. 
    static final double VISION_CORRECTION_FACTOR = 0.1;

    private static Vector2 position;
    private static Vector2 absVelocity;
    private static Vector2 lastAbsVelocity;

    private static boolean vision;

    public static void enableVision(){

    }

    public static void disableVision(){

    }

    public static void init() {
        absVelocity     = new Vector2();
        lastAbsVelocity = new Vector2();
        position        = new Vector2();
        vision = true;
    }
    
    public static void update() {

        // Derive our velocity 
        Vector2 vel = SwerveManager.getRobotDriveVelocity();

        // Rotate our velocity to be local to the field
        vel = vel.rotate(Pigeon.getRotationRad() - Math.PI / 2);

        // Flip the x component of our velocity if we're on the red alliance
        if (DriverStation.getAlliance() == Alliance.Red)
            vel.x *= -1;
        
        lastAbsVelocity = absVelocity; 
        absVelocity = vel;

        // Integrate our velocity to find our position
        position = position.add(absVelocity.add(lastAbsVelocity).mul(0.5 * RTime.deltaTime()));

        if (vision) {
            Vector2 visionPos;
            try {
                visionPos = VisionManager.getPosition();
            } catch (Exception e) {
               System.out.println("VISION SWERVE BAD");
               visionPos = new Vector2();
            }

            
        }
    }

    public static void setPosition(Vector2 newPos) {
        position = newPos;
    }

    public static Vector2 getPosition() {
        return position;
    }
}
