// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.Auto;
import frc.robot.utils.PIDController;

import java.lang.annotation.Target;

import javax.management.MBeanInfo;

import org.opencv.core.RotatedRect;

import com.ctre.phoenix.led.CANdle.VBatOutputMode;

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
  
  private Vector2 target = new Vector2(40,10);
  private double rot = 20;
  private final double DEAD_ZONE = 1;
  private final double SPEED = 0.01;

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
    //SwervePosition.setPosition(new Vector2());

    
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
    SwervePID.setDestPt(target);
    // Auto.bezierCurveAutoTest();
    SwervePID.setDestRot(2*Math.PI);
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

    Vector2 drive = new Vector2(SwervePID.updateOutputX(), SwervePID.updateOutputY()).rotate(Math.PI/2);
    double rotate = SwervePID.updateOutputRot();
    
    /*Vector2 displacement = target.sub(SwervePosition.getPosition());
    double xSwerve = 0;
    double ySwerve = 0;
    
    if(displacement.x > -DEAD_ZONE && displacement.x < DEAD_ZONE){
      xSwerve = 0;
    } else {
      xSwerve = displacement.x;
    }

    if(displacement.y > -DEAD_ZONE && displacement.y < DEAD_ZONE){
      ySwerve = 0;
    } else {
      ySwerve = displacement.y;
    }

    Vector2 move = new Vector2(-xSwerve, ySwerve);
    move.x = ySwerve != 0? move.x/Math.abs(move.x) : 0;
    move.y = ySwerve != 0? move.y/Math.abs(move.y) : 0;

    System.out.println(target.sub(SwervePosition.getPosition()));
    System.out.println(move);*/
   
    SwerveManager.rotateAndDrive(rotate, drive);
    
  }

  /* public double angleRot(Vector2 target){
        double xValue = Swerve target.x;
        double yValue = target.y;
        
        double currentXValue = SwervePosition.getPosition().x;
        double currentYValue = SwervePosition.getPosition().y;

        Math.acos((xValue-currentXValue)/(Math.sqrt(Math.pow(xValue-currentXValue, 2))))
        Math.atan2(yValue, xValue)
    }

    angleRot(target);
    // System.out.println(target.sub(SwervePosition.getPosition()));
    
    */

  @Override
  public void autonomousExit() {

  }

  @Override
  public void teleopInit() 
  {
    OI.init();
    SwervePosition.setPosition(new Vector2());
  }

  @Override
  public void teleopPeriodic() 
  {
    RTime.update();
    SwervePosition.update();
    OI.useInput();
    System.out.println(Pigeon.getRotationRad());
    System.out.println("Position: " + SwervePosition.getPosition().toString());
  }

  // @Override
  // public void disabledInit() {}

  // @Override
  // public void disabledPeriodic() {}

  // @Override
  // public void testInit() {}

  // @Override
  // public void testPeriodic() {}

  // @Override
  // public void simulationInit() 
  // {
    
  // }

  // @Override
  // public void simulationPeriodic() {}
}
