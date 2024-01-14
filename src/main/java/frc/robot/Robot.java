// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.utils.RTime;
import frc.robot.utils.trajectories.BezierCurve;

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
    //NetworkTables.init();
    Telemetry.init();
    SwervePosition.setPosition(new Vector2());
  }

  @Override
  public void robotPeriodic() {
    Pigeon.update();
    RTime.updateAbsolute();
    Telemetry.update(false);
  }
 
  @Override
  public void autonomousInit() {
    RTime.init();
    Pigeon.setYaw(270);
  }

  @Override
  public void autonomousPeriodic() {
    SwervePosition.update();
    RTime.update();
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
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
