package frc.robot.autoframe;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.BezierCurve;
import static frc.robot.utils.Auto.movement;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Tuning;

public class FollowBezierCurve extends Autoframe{
    public BezierCurve trajectory;
    PIDController trajectoryPID, rotPID;
    CurveAutoFrame[] autoFrames;

    public FollowBezierCurve(BezierCurve trajectory, CurveAutoFrame[] autoFrames) {
        this.trajectory = trajectory;
        blocking = true;
        this.rotPID = SwervePID.rotPID;
        rotPID.setDest(trajectory.rotEnd);
    }

    @Override
    public void start() {
        this.trajectoryPID = new PIDController(Tuning.SWERVE_TRL_P, Tuning.SWERVE_TRL_I, Tuning.SWERVE_TRL_D, 1.0, 1.0, 1.0);
        this.trajectoryPID.setDest(1.0);
        SwervePID.setDestRot(trajectory.rotEnd);
    }   

    @Override
    public void update() {
        double t = trajectory.getClosestT(SwervePosition.getPosition());
        Vector2 movementVector = trajectory.getTangent(t);
        movementVector = movementVector.norm();
        double translationSpeed = trajectoryPID.updateOutput(t);

        if (RobotBase.isSimulation()) {
            translationSpeed *= 0.05;
            movement = movementVector.mul(translationSpeed);

        }
        else {
            movement = movementVector.rotate(Math.PI/2.0).mul(translationSpeed);
        }

        if (t > 0.97) {
            movement = new Vector2();
            SwerveManager.rotateAndDrive(0.0, new Vector2());
            this.done = true;
        }else{
            SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), movementVector.rotate(Math.PI/2.0).mul(translationSpeed));
        }
    }
}
