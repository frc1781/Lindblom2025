// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends LoggedRobot {
  private static XboxController controller = new XboxController(0); 
  private static final int motorID = 22;
  private SparkMax motor;
  private double setpoint = 0;
  private double kP = 0.00001;
  private double kI = 0;
  private double kD = 0;
  private double FF = 1/10000.0;
  private double minOutput = -10000;
  private double maxOutput = 10000;
  private double maxVelocity = 10500;
  private double maxAcceleration = 5000;


  Robot () {
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());

      new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
    //Spark max configuration done here, including PID, reference ARM in 2025 on how to do that...
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    motorConfig.smartCurrentLimit(30);
    motorConfig.closedLoop.pid(kP, kI, kD);
    motorConfig.closedLoop.velocityFF(FF); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
    motorConfig.closedLoop.outputRange(minOutput, maxOutput);
    motorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
    motorConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    motorConfig.closedLoop.maxMotion.maxVelocity(maxVelocity);
    motorConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
    motorConfig.closedLoop.maxMotion.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void robotPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setpoint = -controller.getLeftY() * 10000;
    maxAcceleration = controller.getRightTriggerAxis();
    motor.getClosedLoopController().setReference(setpoint, ControlType.kMAXMotionVelocityControl);
    Logger.recordOutput("motorID", motorID);
    Logger.recordOutput("setpoint", setpoint);
    Logger.recordOutput("kP", kP);
    Logger.recordOutput("kI", kI);
    Logger.recordOutput("kD", kD);
    Logger.recordOutput("FF", FF);
    //Logger.recordOutput("kS", kS);
    //Logger.recordOutput("kG", kG);
    //Logger.recordOutput("kA", kA);
    //Logger.recordOutput("kV", kV);
    Logger.recordOutput("minOutput", minOutput);
    Logger.recordOutput("maxOutput", maxOutput);
    Logger.recordOutput("maxVelocity", maxVelocity);
    Logger.recordOutput("maxAcceleration", maxAcceleration);
    Logger.recordOutput("motorSpeed", motor.getEncoder().getVelocity());
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
