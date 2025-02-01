package tech.lindblom.control;

import tech.lindblom.subsystems.drive.DriveController;

public class TestManager {

    RobotController robotController;
    DriveController driveController;

    public TestManager(RobotController robotController) {
        this.robotController = robotController;
        driveController = robotController.driveController;
    }
    
    void testInit() {

    }

    void testRun() {

    }

}
