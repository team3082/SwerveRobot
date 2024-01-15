// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.BezierCurve;
import frc.robot.utils.RTime;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {

    Pigeon.init();
    Pigeon.zero();
    SwerveManager.init();
    SwervePosition.init();
    SwervePID.init();
    OI.init();
    Pigeon.setYaw(270);
    Telemetry.init();
    SwervePosition.setPosition(new Vector2());
  }

  @Override
  public void robotPeriodic() {
    Pigeon.update();
    RTime.updateAbsolute();
    Telemetry.update(false);
  }
 
  BezierCurve trajectory;
  PIDController trajectoryPID;
  PIDController rotPID;
  @Override
  public void autonomousInit() {
    RTime.init();
    Pigeon.setYaw(270);
    trajectory = new BezierCurve(new Vector2(33, -149), new Vector2(101.6, -106), new Vector2(-87.5, -67), new Vector2(-17, -26), 0.0, 1, new Vector2(1, 1), 1.0);
    trajectoryPID = new PIDController(0.1, 0.001, 0.03, 1.0, 1.0, 0.25);
    trajectoryPID.setDest(1);
    rotPID = new PIDController(0.1, 0.001, 0.03, 1.0, 1.0, 0.0005);
    rotPID.setDest(trajectory.rotEnd);
    SwervePosition.setPosition(trajectory.a);
    Pigeon.setYaw(Math.toDegrees(trajectory.rotStart));
    System.out.println(trajectoryPID.getDest());
  }


  double t;
  double translationSpeed;
  double rotSpeed;
  Vector2 txy;
  Vector2 movementVector;
  @Override
  public void autonomousPeriodic() {
    SwervePosition.update();
    RTime.update();
    t = trajectory.getClosestT(SwervePosition.getPosition());
    txy = trajectory.getPoint(t);
    movementVector = trajectory.getTangent(t);
    movementVector = movementVector.norm();
    translationSpeed = trajectoryPID.updateOutput(t);
    // rotSpeed = rotPID.updateOutput(Pigeon.getRotationRad());
    SwervePID.setDestRot(trajectory.rotEnd);
    SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), movementVector.mul(translationSpeed * 0.05));
    System.out.println(" desired angle: " + trajectory.rotEnd + " currentAngle: " + Pigeon.getRotationRad() + " rotSpeed: " + rotSpeed);
    // System.out.println("Current t: " + t + " Current Error: " + trajectoryPID.getError() + "Percent Speed: " + translationSpeed);
    // System.out.println("Movement Vector: " + movementVector.norm() + " Swerve Position: " + SwervePosition.getPosition());
  }

  @Override
  public void autonomousExit() {

  }

  @Override
  public void teleopInit() {
    OI.init();
    SwervePosition.setPosition(new Vector2());
  }

  @Override
  public void teleopPeriodic() {
    RTime.update();
    SwervePosition.update();
    OI.useInput();
    System.out.println(Pigeon.getRotationRad());
    System.out.println("Position: " + SwervePosition.getPosition().toString());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    
  }

  @Override
  public void simulationPeriodic() {}
}
