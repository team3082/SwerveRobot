package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controls.ControlReference;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePID.PIDType;
import frc.robot.utilities.Vector2;
import frc.robot.utilities.Math.RMath;

public class OI {
    public static Joystick driverStick;

    static final int moveX     = ControlReference.AXIS_LEFT_X;
    static final int moveY     = ControlReference.AXIS_LEFT_Y;
    static final int rotateX   = ControlReference.AXIS_RIGHT_X;
    static final int boost     = ControlReference.AXIS_RIGHT_TRIGGER;
    static final int zero      = ControlReference.BUTTON_Y;
    static final int field     = ControlReference.BUTTON_X;
    static final int cancel    = ControlReference.BUTTON_A;

    static boolean f = true;
    static PIDType PID = PIDType.NONE;

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
        PID = PIDType.NONE;
    }

    /**
     * Instruct the robot to follow instructions from joysticks.
     * One call from this equals one frame of robot instruction.
     * Because we used TimedRobot, this runs 50 times a second,
     * so this lives in the teleopPeriodic() function.
     */
    public static void useInput() {

        if (driverStick.getRawButton(zero)) Pigeon.zero();
        if (driverStick.getRawButton(cancel)) PID = PIDType.NONE;

        double kBoostCoefficient = 0.1;

        if (driverStick.getRawAxis(boost) > .5) kBoostCoefficient = 1;

        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX), -driverStick.getRawAxis(moveY));
        double rotate = RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -0.3;

        if (drive.mag() < 0.125)
            drive = new Vector2();
        else
            drive = RMath.smoothJoystick2(drive).mul(kBoostCoefficient);

        if (Math.abs(rotate) < 0.005) {
            rotate = 0;
            int POV = driverStick.getPOV();
            if(POV != -1) {
                PID = PIDType.ROTATION;
                SwervePID.setDestRot(Math.PI / 2.0 - Math.toRadians(POV - 180));
            }
        } else {
            PID = PIDType.NONE;
        }

        if (driverStick.getRawButton(field)) {
            for (SwerveModule module: SwerveManager.swerveModules) {
                module.rotate((module.pos.atan2()));
            }
        }

        System.out.println("Rotate: " + rotate + " Drive: " + drive.toString());

        // Swerving and a steering! Zoom!
        SwerveManager.rotateAndDrive(rotate, drive);
    }
}
