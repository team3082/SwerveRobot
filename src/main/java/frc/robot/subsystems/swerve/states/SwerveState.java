package frc.robot.subsystems.swerve.states;

public class SwerveState {
    
    public double velocity;
    public double rotation;

    public SwerveState(double vel, double rot) {
        this.velocity = vel;
        this.rotation = rot;
    }

    public SwerveState() {
        this.velocity = 0;
        this.rotation = 0;
    }

    public double getPos() {
        return velocity;
    }

    public double getRot() {
        return rotation;
    }
}
