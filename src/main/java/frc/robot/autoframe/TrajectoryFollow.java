package frc.robot.autoframe;

import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.utils.followers.SwerveFollower;

public class TrajectoryFollow extends Autoframe{
    public SwerveFollower controller;
    public double tStart;

    public TrajectoryFollow(SwerveFollower follower){
        this.controller = follower;
    }

    public void start(){
        tStart = RTime.now();
        
    }

    public void update(){
        Vector2 currentPos = SwervePosition.getPosition();
        Vector2 currentVel = SwerveManager.getRobotDriveVelocity();
        double currentAng = Pigeon.getRotationRad();
        double currengAngVel = Pigeon.getDeltaRotRad();
        SwerveState currentState = new SwerveState(currentPos, currentAng, currentVel, currengAngVel);
        SwerveManager.rotateAndDrive(controller.getInstruction(currentState, RTime.now() - tStart));
        if(controller.path.endState().getPos().dist(currentPos) < 0.1){
            done = true;
        }
    }

    
}
