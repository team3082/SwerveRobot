package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.utils.RTime;

public class Pigeon {

    public static WPI_Pigeon2 pigeon;
    private static double lastRot;
    private static double deltaRot;

    private static double simulatedRot = 0;

    public static void init() {
        pigeon = new WPI_Pigeon2(0);
        pigeon.configFactoryDefault();
    }

    public static void update() {
        if (RobotBase.isSimulation()) {
            simulatedRot += SwerveManager.getRotationalVelocity() * RTime.deltaTime();
        }
        deltaRot = (getRotationRad() - lastRot) / RTime.deltaTime();
        lastRot = getRotationRad();
    }

    public static void setSimulatedRot(double rad) {
        simulatedRot = rad;
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
        if (RobotBase.isSimulation()) {
            return simulatedRot;
        }
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
        return deltaRot;
    }
}
