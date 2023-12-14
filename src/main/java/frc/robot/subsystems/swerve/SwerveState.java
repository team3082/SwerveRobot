package frc.robot.subsystems.swerve;

import frc.robot.utils.Vector2;

public class SwerveState {
    final Vector2 pos;
    final double heading;

    public SwerveState(Vector2 pos, double heading){
        this.pos = pos;
        this.heading = heading;
    }
}
