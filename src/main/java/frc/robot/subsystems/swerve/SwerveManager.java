package frc.robot.subsystems.swerve;

import frc.robot.utils.Vector2;

public class SwerveManager {
    /**
     * Array containing all four swerve modules.
     */
    public static SwerveModule[] swerveModules;

    public static void init() {
        swerveModules = new SwerveModule[] {
            new SwerveModule(1, 2, -1, 1, 143.877),
            new SwerveModule(3, 4, 1, 1, 266.221),
            new SwerveModule(5, 6, 1, -1, 180.352),
            new SwerveModule(7, 8, -1, -1, 54.229),
        };

        SwerveKinematics.init(swerveModules);
    }

    /**
     * Zero the encoders for all of the swerve modules.
     */
    public static void zeroEncoders() {
        for (SwerveModule module : swerveModules) {
            module.resetEncoder();
        }
    }

    /**
     * Set the states of the SwerveModules.
     * @param move The raw movement vector, derived from the joystick
     * @param rot The raw rotation value, derived from the joystick
     */
    public static void setStates(Vector2 move, double rot) {
        SwerveKinematics.calculateStates(rot, move);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setModuleState(SwerveKinematics.finalStates[i]);
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
    public static double getEncoderPos(int id) {
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