// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
//import tech.lindblom.subsystems.drive.DriveController;
//import tech.lindblom.subsystems.led.LEDs;
//import tech.lindblom.subsystems.types.StateSubsystem;
//import tech.lindblom.subsystems.types.Subsystem;
//import tech.lindblom.utils.Constants;
//import tech.lindblom.utils.EnumCollection;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private static XboxController controller = new XboxController(0); 
  private static final int motorID = 22;
  private SparkMax motor; 

  Robot () {
    motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
    //Spark max configuration done here, including PID, reference ARM in 2025 on how to do that...
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    armMotorConfig.smartCurrentLimit(30);
    armMotorConfig.absoluteEncoder.positionConversionFactor(360.0);
    armMotorConfig.closedLoop.pid(0.01, 0,0.001);
    armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
    armMotorConfig.closedLoop.outputRange(-0.5, 0.5);
    armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
    armMotorConfig.closedLoop.maxMotion.maxAcceleration(0.0000001);
    armMotorConfig.closedLoop.maxMotion.maxVelocity(0.01);
    armMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
    armMotorConfig.closedLoop.maxMotion.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);
    motor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void robotPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    motor.getClosedLoopController().setReference(4, ControlType.kVoltage);
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
