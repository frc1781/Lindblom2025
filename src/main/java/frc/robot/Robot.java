// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import tech.lindblom.Config;
import tech.lindblom.subsystems.Subsystem;
import tech.lindblom.utils.DriverInput;
import tech.lindblom.control.ControlSystem;

public class Robot extends LoggedRobot {
  private DriverInput mDriverInput;
  private ControlSystem mControlSystem;

  @Override
  public void robotInit() {
    mDriverInput  = new DriverInput();
    mControlSystem = new ControlSystem();
    Logger.recordMetadata("RobotName", "GLaDOS");
    Logger.recordMetadata("Team", "1781");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());

      new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      setUseTiming(false);
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    mDriverInput.addClickListener(Config.DRIVER_CONTROLLER_PORT, Config.RESET_NAVX, (isPressed -> {
      if (isPressed) {
        mControlSystem.resetNavX();
      }
    }));
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    mControlSystem.init(Subsystem.OperatingMode.AUTONOMOUS);
  }

  @Override
  public void autonomousPeriodic() {
    mControlSystem.run(null);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mControlSystem.init(Subsystem.OperatingMode.TELEOP);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mControlSystem.run(mDriverInput.run());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
