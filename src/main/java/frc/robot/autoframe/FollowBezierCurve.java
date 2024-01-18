package frc.robot.autoframe;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.utils.constructors.trajectories.BezierCurve;
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
    }

    @Override
    public void start() {
        this.trajectoryPID = new PIDController(Tuning.SWERVE_TRL_P, Tuning.SWERVE_TRL_I, Tuning.SWERVE_TRL_D, 1.0, 1.0, 1.0);
        this.trajectoryPID.setDest(this.trajectory.length());
        SwervePID.setDestRot(trajectory.rotEnd);
    }   

    @Override
    public void update() {
        // get odometry position data
        Vector2 robotPos = SwervePosition.getPosition();

        // get the closest point t on the Bezier Curve
        double t = this.trajectory.getClosestT(robotPos);

        // gets the length traveled on the curve for the PID Controller
        double lengthTraveled = this.trajectory.getLengthTraveled(robotPos);

        // Derive our movement vector from the curve
        Vector2 movementVector = this.trajectory.getTangent(t, robotPos);

        // Calculate our speed from where we are along the curve
        // (slow down closer we get to the finish)
        double translationSpeed = this.trajectoryPID.updateOutput(lengthTraveled);

        // If we are within our deadband
        if (t > 0.97) {
            // Kill the drivetrain if we are in simulation
            if (RobotBase.isSimulation()) {
                movement = new Vector2();
                SwerveManager.rotateAndDrive(0.0, new Vector2());
            }
            else {
                movement = new Vector2();
            }
            this.done = true;
        }else{
            // SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), movementVector.rotate(Math.PI/2.0).mul(translationSpeed));
            if (RobotBase.isSimulation()) {
                // Slow down if we are in simulation because we go zoom zoom
                translationSpeed *= 0.05;
                movement = movementVector.mul(translationSpeed);
                SwerveManager.rotateAndDrive(0.0, movementVector.mul(translationSpeed));
                System.out.println(movementVector);
            }
            else {
                // Rotate our vector to be local to the field
                // and scale for our current speed.
                movement = movementVector.rotate(Math.PI / 2.0).mul(translationSpeed);
            }
        }
    }
}
