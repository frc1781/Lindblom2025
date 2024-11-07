// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import tech.lindblom.control.RobotController;
import tech.lindblom.utils.EnumCollection;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotController controlSystem;


  @Override
  public void robotInit() {
    controlSystem = new RobotController();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
    controlSystem.init(EnumCollection.OperatingMode.AUTONOMOUS);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    controlSystem.run(EnumCollection.OperatingMode.AUTONOMOUS);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    controlSystem.init(EnumCollection.OperatingMode.TELEOP);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controlSystem.run(EnumCollection.OperatingMode.TELEOP);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    controlSystem.init(EnumCollection.OperatingMode.DISABLED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    controlSystem.run(EnumCollection.OperatingMode.DISABLED);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    controlSystem.init(EnumCollection.OperatingMode.TEST);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    controlSystem.run(EnumCollection.OperatingMode.TEST);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
