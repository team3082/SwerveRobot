package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.utils.RTime;
import frc.robot.utils.Vector3;

public class Pigeon {

    public static WPI_Pigeon2 pigeon;
    private static double lastRot;
    private static double rotVel;

    public static void init() {
        pigeon = new WPI_Pigeon2(0);
        pigeon.configFactoryDefault();
    }

    public static void update() {
        rotVel = (getRotationRad() - lastRot) / RTime.deltaTime();
        lastRot = getRotationRad();
    }

    public static void zero(){
        pigeon.setYaw(90);
    }

    public static void setYaw(double deg) {
        pigeon.setYaw(deg);
    }

    public static void setYawRad(double rad) {
        setYaw(Math.PI * rad / 180.0);
    }

    
    /**
     * Local to the robot, not the world
     * Pitch, rotates around the X, left to right, axis
     * Tilts forward and backward
     * @return Pitch in radians
     */
    public static double getPitchRad() {
        return Math.PI * pigeon.getPitch() / 180;
    }

    /**
     * Local to the robot, not the world
     * Yaw, rotates around the Y, up and down, axis
     * @return Yaw in radians
     */
    public static double getRotationRad() {
        return Math.PI * pigeon.getYaw() / 180;
    }

    /**
     * Local to the robot, not the world
     * Roll, rotates around the Z, forward and backward, axis
     * Tilts left and right
     * @return Roll in radians
     */
    public static double getRollRad() {
        return Math.PI * pigeon.getRoll() / 180;
    }

    /**
     * Gets the rotation speed (yaw) of the robot in radians per second
     * @return Yaw in radians
     */
    public static double getDeltaRotRad() {
        return rotVel;
    }

    /**
     * Returns the unit vector in the direction of the z-axis relative to the pigeon
     * This vector is in reference to the basis vectors of the pigeon at the start of the competition
     * @return The unit vector in the direction of the z-axis relative to the pigeon
     */
    public static Vector3 getKHat(){

        double yaw = getRotationRad();
        double roll = getRollRad();
        double pitch = getPitchRad();

        double x,y,z;

        x = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) + (Math.sin(yaw) * Math.sin(roll));
        y = Math.cos(yaw) * Math.sin(roll) * -1  + (Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll));
        z = Math.cos(pitch) * Math.cos(roll);
        return new Vector3(x,y,z);
        
    }
}
