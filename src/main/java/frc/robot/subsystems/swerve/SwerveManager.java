package frc.robot.subsystems.swerve;

import frc.robot.subsystems.Pigeon;
import frc.robot.utils.*;

public class SwerveManager {

    /**
     * Array containing all four swerve modules.
     */
    public static SwerveModule[] swerveModules;

    public static void init() {
        swerveModules = new SwerveModule[] {
            new SwerveModule(5, 6, 1, -1, 289.424), // BR
            new SwerveModule(7, 8, -1, -1, 95.537), // BL
            new SwerveModule(1, 2, -1, 1, 249.521), // FL
            new SwerveModule(3, 4, 1, 1, 208.213), // FR
        };
       swerveModules[0].driveMotor.setInverted(true);
    }

    /**
     * Zero the encoders for all of the swerve modules.
     */
    public static void zeroEncoders() {
        for (SwerveModule module : swerveModules) {
            module.resetSteerEncoder();
        }
    }

    public static void rotateAndDrive(SwerveInstruction instruction) {
        rotateAndDrive(instruction.rotation, instruction.movement);
    }

    /**
     * Simultaneously rotate and drive all four of the swerve modules.
     * @param rotSpeed Speed the steer motor should drive at
     * @param move Vector representing desired robot output.
     */
    public static void rotateAndDrive(double rotSpeed, Vector2 move) {

        double heading = Pigeon.getRotationRad();
        
        // Array containing the unclamped movement vectors of each module
        Vector2[] vectors = new Vector2[swerveModules.length];

        // Multiply the movement vector by a rotation matrix to compensate for the pigeon's heading
        Vector2 relMove = move.rotate(-(heading - Math.PI / 2));

        // The greatest magnitude of any module's distance from the center of rotation
        double maxModPosMagnitude = 0;
        for (int i = 0; i < swerveModules.length; i++) {
            maxModPosMagnitude = Math.max(maxModPosMagnitude,
                    swerveModules[i].pos.mag());
        }

        // The greatest speed of the modules. If any one module's speed is
        // greater than 1.0, all the speeds are scaled down.
        double maxSpeed = 1.0;

        // Calculate unclamped movement vectors
        for (int i = 0; i < swerveModules.length; i++) {
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector according to maxModPosMagnitude (such that the magnitude of
            // the largest vector is 1), and scaling it by a factor of rotSpeed.

            Vector2 rotate = new Vector2(
                (-1 * swerveModules[i].pos.y / maxModPosMagnitude) * rotSpeed,
                (     swerveModules[i].pos.x / maxModPosMagnitude) * rotSpeed);

            // The final movement vector, calculated by summing movement and rotation
            // vectors

            Vector2 rotMove = relMove.add(rotate);

            vectors[i] = rotMove;
            maxSpeed = Math.max(maxSpeed, rotMove.mag());
        }

        for (int i = 0; i < swerveModules.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. 
            double direction = vectors[i].atan2();
            double power = vectors[i].mag() / maxSpeed;

            // Drive the swerve modules
            if(power != 0)
                swerveModules[i].rotateToRad(direction);
            swerveModules[i].drive(power);
        }
    }

    /**
     * Returns the overall rotational velocity of the robot, based on the rotations and velocities of each of the
     * wheels. Primarily meant for applications where the Pigeon is unavailable, such as simulation.
     * @return the rotational velocity of the robot, in radians/second
     */
    public static double getRotationalVelocity() {
        // We only need to check the first swerve module
        Vector2 moduleVel = Vector2.fromPolar(getSteerAngle(0), getDriveVelocity(0));
        Vector2 rotVector = moduleVel.sub(getRobotDriveVelocity());

        // Get the one-dimensional rotation velocity in inches/sec, moving around the circle
        double rotVelInches = rotVector.rotate(-swerveModules[0].pos.atan2()).y;

        // Divide by the radius to get the rotation velocity in radians/sec
        double radius = swerveModules[0].pos.mag();
        return rotVelInches / radius;
    }


    /**
     * Gets the raw encoder position of a specified SwerveMod's drive motor
     * @param id the ID of the SwerveModule to check
     * @return the raw encoder position, in ticks
     */
    public static double getEncoderPos(int id){
        return swerveModules[id].driveMotor.getEncoder().getPosition();
    }

    /**
     * Gets the velocity a given SwerveModule is driving at
     * @param id the ID of the SwerveModule to check
     * @return the drive velocity of the SwerveModule, in inches/second
     */
    public static double getDriveVelocity(int id) {
        return swerveModules[id].getDriveVelocity();
    }

    /**
     * Returns the angle a given SwerveModule's wheel is pointed toward
     * @param id the ID of the SwerveModule to check
     * @return the angle of the SwerveModule, in radians
     */
    public static double getSteerAngle(int id) {
        return swerveModules[id].getSteerAngle();
    }

    /**
     * Returns the overall drive velocity of the robot, based on the average of the velocities of the wheels. Not
     * adjusted for the rotation of the robot on the field.
     * @return the robot's drive velocity, in inches/second
     */
    public static Vector2 getRobotDriveVelocity() {
        Vector2 velSum = new Vector2();

        for (SwerveModule mod : swerveModules) {
            velSum = velSum.add(Vector2.fromPolar(mod.getSteerAngle(), mod.getDriveVelocity()));
        }

        return velSum.div(swerveModules.length);
    }
}