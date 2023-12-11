package frc.robot.subsystems.swerve;

import frc.robot.utils.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.swerve.states.SwerveInstruction;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;

public class SwervePID {

    public static PIDController xPID, yPID, rotPID;

    public static double moveP = 0.018;
    public static double moveI = 0;
    public static double moveD = 0.0005;
    public static double moveDead = 1;
    public static double moveVelDead = 0.2;
    public static double moveSpeedMax = 0.3;

    public static double rotP = 0.825;
    public static double rotI = 0.005;
    public static double rotD = 0.002;
    public static double rotDead = 0.025;
    public static double rotVelDead = 0.1;
    public static double rotSpeedMax = 0.3;

    public static void init() {
        xPID = new PIDController(moveP, moveI, moveD, moveDead, moveVelDead, moveSpeedMax);
        yPID = new PIDController(moveP, moveI, moveD, moveDead, moveVelDead, moveSpeedMax);
        rotPID = new PIDController(rotP, rotI, rotD, rotDead, rotVelDead, rotSpeedMax);
    }

    public static void setDestState(Vector2 dest, double destRot) {
        setDestPt(dest);
        setDestRot(destRot);
    }

    public static void setDestPt(Vector2 dest) {
        setDestX(dest.x);
        setDestY(dest.y);
    }

    public static void setDestX(double dest) {
        xPID.setDest(dest);
    }

    public static void setDestY(double dest) {
        yPID.setDest(dest);
    }

    public static void setDestRot(double dest) {
        rotPID.setDest(RMath.targetAngleAbsolute(Pigeon.getRotationRad(), dest, 2*Math.PI));
    }
    
    public static double updateOutputX() {
        return (xPID.atSetpoint()? 0 : (DriverStation.getAlliance()==Alliance.Red?-1:1)*xPID.updateOutput(SwervePosition.getPosition().x));
    }

    public static double updateOutputY() {
        return yPID.atSetpoint()? 0 : yPID.updateOutput(SwervePosition.getPosition().y);
    }

    public static double updateOutputRot() {
        return rotPID.updateOutput(Pigeon.getRotationRad());
    }

    public static Vector2 updateOutputVel() {
        return new Vector2(updateOutputX(), updateOutputY());
    }

    public static SwerveInstruction updateAll() {
        return new SwerveInstruction(updateOutputRot(), updateOutputVel());
    }

    public static Vector2 getDest() {
        return new Vector2(xPID.getDest(), yPID.getDest());
    }

    public static boolean atDest() {
        return xPID.atSetpoint() && yPID.atSetpoint();
    }
    
    public static boolean atRot() {
        return rotPID.atSetpoint();
    }

}