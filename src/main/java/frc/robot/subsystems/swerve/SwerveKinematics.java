package frc.robot.subsystems.swerve;

import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.utils.Vector2;

public class SwerveKinematics {
    
    public static SwerveModule[] targetedModules;
    /** The final, normalized SwerveModule states. */
    public static SwerveState[] finalStates = new SwerveState[] {
        new SwerveState(),
        new SwerveState(),
        new SwerveState(),
        new SwerveState()
    };

    public static void init(SwerveModule[] mods) {
        targetedModules = mods;
    }

    /**
     * Calculates the desired states of the SwerveModules,
     * in a clockwise fashion, starting from the top-left module.
     * @param rotation Desired rotation to normalize
     * @param move Desired movement vector to normalize
     * @return Array of normalized SwerveModule states
     */
    public static void calculateStates(double rotation, Vector2 move) {

        double heading = Pigeon.getRotationRad();
        
        // Array containing the unclamped movement vectors of each module
        Vector2[] vectors = new Vector2[targetedModules.length];

        // Multiply the movement vector by a rotation matrix to compensate for the pigeon's heading
        Vector2 relMove = move.rotate(-(heading - Math.PI / 2));

        // The greatest magnitude of any module's distance from the center of rotation
        double maxModPosMagnitude = 0;
        for (int i = 0; i < targetedModules.length; i++) {
            maxModPosMagnitude = Math.max(maxModPosMagnitude,
                targetedModules[i].pos.mag());
        }

        // The greatest speed of the modules. If any one module's speed is
        // greater than 1.0, all the speeds are scaled down.
        double maxSpeed = 1.0;

        // Calculate unclamped movement vectors
        for (int i = 0; i < targetedModules.length; i++) {
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector according to maxModPosMagnitude (such that the magnitude of
            // the largest vector is 1), and scaling it by a factor of rotSpeed.

            Vector2 rotate = new Vector2(
                (-1 * targetedModules[i].pos.y / maxModPosMagnitude) * rotation,
                (     targetedModules[i].pos.x / maxModPosMagnitude) * rotation);

            // The final movement vector, calculated by summing movement and rotation
            // vectors

            Vector2 rotMove = relMove.add(rotate);

            vectors[i] = rotMove;
            maxSpeed = Math.max(maxSpeed, rotMove.mag());
        }

        for (int i = 0; i < targetedModules.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. 
            double direction = vectors[i].atan2();
            double power = vectors[i].mag() / maxSpeed;

            if (power != 0)
                finalStates[i].rotation = direction;
            finalStates[i].velocity = power;
        }
    }
}
