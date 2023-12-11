package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;
import frc.controllermaps.*;

public class OI {
    public static Joystick driverStick;

    static final int moveX     = LogitechF310.AXIS_LEFT_X;
    static final int moveY     = LogitechF310.AXIS_LEFT_Y;
    static final int rotateX   = LogitechF310.AXIS_RIGHT_X;
    static final int boost     = LogitechF310.AXIS_RIGHT_TRIGGER;
    static final int slow     = LogitechF310.AXIS_LEFT_TRIGGER;
    static final int zero      = LogitechF310.BUTTON_Y;
    static final int lock     = LogitechF310.BUTTON_X;
    static final int cancel    = LogitechF310.BUTTON_A;

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
    }

    /**
     * Instruct the robot to follow instructions from joysticks.
     */
    public static void useInput() {

        if (driverStick.getRawButton(zero)) Pigeon.zero();

        double kBoostCoefficient = 0.1;

        if (driverStick.getRawAxis(boost) > .5) kBoostCoefficient = 1;

        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX), -driverStick.getRawAxis(moveY));
        double rotate = RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -0.3;

        if (driverStick.getRawAxis(slow) > .5) rotate *= 0.3;

        if (drive.mag() < 0.125)
            drive = new Vector2();
        else
            drive = RMath.smoothJoystick2(drive).mul(kBoostCoefficient);

        if (Math.abs(rotate) < 0.005) rotate = 0;

        if (driverStick.getRawButton(lock)) {
            for (SwerveModule module: SwerveManager.swerveModules) {
                module.rotate((module.pos.atan2()));
            }
        }

        System.out.println("Rotate: " + rotate + " Drive: " + drive.toString());

        // Swerving and a steering! Zoom!
        SwerveManager.setStates(drive, rotate);
    }
}