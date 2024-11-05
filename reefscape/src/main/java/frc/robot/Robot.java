package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends LoggedRobot {

  @Override
  public void robotInit() {
    robotController.init();
  }

  @Override
  public void robotPeriodic() {
    robotController.periodic();
  }

  @Override
  public void autonomousInit() {
    autonomousControl.autonomousInit();
    robotController.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {
    robotController.periodic(autonomousControl.update(timestamp));  //returns input to robot as if it was the driver's station
  }

  @Override
  public void teleopInit() {
    driverStation.telopInit();
    robotController.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    robotController.periodic(driverStation.getCurrentInput(timestamp)); //returns input to robot, robot can not tell if in auto or tele 
  }

  @Override
  public void disabledInit() {
    robotController.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    robotController.disabledPeriodic();
  }

  @Override
  public void testInit() {
    robotController.testInit();
  }

  @Override
  public void testPeriodic() {
    robotController.testPeriodic(testInput);
  }

  @Override
  public void simulationInit() {
    robotController.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    robotController.periodic(simulatedInput);
  }
}
