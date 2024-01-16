package frc.robot.autoframe;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.BezierCurve;
import static frc.robot.utils.Auto.movement;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.Tuning;

public class FollowBezierCurve extends Autoframe{
    BezierCurve trajectory;
    PIDController trajectoryPID;

    public FollowBezierCurve(BezierCurve trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public void start() {
        SwervePosition.setPosition(this.trajectory.a);
        this.trajectoryPID = new PIDController(Tuning.SWERVE_TRL_P, Tuning.SWERVE_TRL_I, Tuning.SWERVE_TRL_D, 1.0, 1.0, 0.75);
        this.trajectoryPID.setDest(1.0);
    }   

    @Override
    public void update() {
        double t = trajectory.getClosestT(SwervePosition.getPosition());
        // use to correct path (if needed)
        // Vector2 txy = trajectory.getPoint(t);
        Vector2 movementVector = trajectory.getTangent(t);
        movementVector = movementVector.norm();
        double translationSpeed = trajectoryPID.updateOutput(t);

        if (RobotBase.isSimulation()) {
            translationSpeed *= 0.05;
        }

        SwerveManager.rotateAndDrive(0.0, movementVector.rotate(Math.PI/2.0).mul(translationSpeed));

        if (t == 1.0) {
            movement = new Vector2();
            this.done = true;
        }
    }
}
