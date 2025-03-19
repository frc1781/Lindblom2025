package tech.lindblom.control;
import java.util.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.subsystems.arm.Arm;
import tech.lindblom.subsystems.auto.Auto;
import tech.lindblom.subsystems.auto.AutoStep;
import tech.lindblom.subsystems.auto.routines.CenterOneCoral;
import tech.lindblom.subsystems.auto.routines.Collect;
import tech.lindblom.subsystems.auto.routines.LeftFourCoral;
import tech.lindblom.subsystems.auto.routines.LeftOneCoral;
import tech.lindblom.subsystems.auto.routines.LeftThreeCoral;
import tech.lindblom.subsystems.auto.routines.RightFourCoral;
import tech.lindblom.subsystems.auto.routines.RightOneCoral;
import tech.lindblom.subsystems.auto.routines.RightThreeCoral;
import tech.lindblom.subsystems.auto.routines.TestRoutine;
import tech.lindblom.subsystems.climber.BaseClimber;
import tech.lindblom.subsystems.climber.Climber;
import tech.lindblom.subsystems.climber.ClimberSim;
import tech.lindblom.subsystems.conveyor.Conveyor;
import tech.lindblom.subsystems.thumb.Thumb;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.elevator.Elevator;
import tech.lindblom.subsystems.led.LEDs;
import tech.lindblom.subsystems.led.LEDs.LEDState;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.subsystems.vision.Vision;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection;

public class TestController {
    RobotController robotController;
    DriveController driveController;
    DriverInput.TestInputHolder testInputs;
    boolean stepStarted;
    boolean currentStepPassed = false;
    String currentMsg = "";
    private int testStep = 0;
    private boolean nextTestStepPushed = false;
    private boolean prevTestStepPushed = false;

     public TestController(RobotController robotController) {
        this.robotController = robotController;
        this.driveController = robotController.driveController;
     }

     private void ifStartingStep() {
        stepStarted = false;
        if (testInputs.nextTest && !nextTestStepPushed) {
            nextTestStepPushed = true;
            testStep++;
            stepStarted = true;
        }
        nextTestStepPushed = testInputs.nextTest;
        if (testInputs.prevTest && !prevTestStepPushed) {
            prevTestStepPushed = true;
            stepStarted = true;
            testStep--;
        }
        prevTestStepPushed = testInputs.prevTest;  

        if (stepStarted) {
            currentStepPassed = false;
        }
     }

     public void periodic(DriverInput.TestInputHolder testInputs) {
        this.testInputs = testInputs;
        System.out.println(this.testInputs.nextTest);
        ifStartingStep();
        switch(testStep) {
            case 0:
                currentMsg = "Starting testing, hit B for first test";
                currentStepPassed = true; //passes by default
                break;
            case 1:
                currentMsg = "Test 1: Testing leds, they should be green";
                currentStepPassed = true;  //passes by default but check leds are green
                break;
            case 2:
                currentMsg = "Test 2: Testing front Time of Flights, put your hands in front of both";
                currentStepPassed = driveController.testFrontTOFs();
                break;
            case 3:
                currentMsg = "Test 3: Testing Arm Time of Flight, put your hand in front of it";
                currentStepPassed = driveController.testArmTOF();
                break;
            case 4:
                currentMsg = "Test 4: don't know??";
                break;
            case 5:
                currentMsg = "Test 5: Testing Coral Time of Flight, put a coral into the arm";
                currentStepPassed = robotController.armSystem.testHasCoral();
                break;
            case 6:
                currentMsg = "Test 6: Testing Conveyer Beambreaks, put your hand in front of each";
                currentStepPassed = false;
                break;
            case 7:
                currentMsg = "Test 7: Testing Cameras, put an apriltag in front of each camera and cover the other cameras not being tested";
                currentStepPassed = false;
                break;
            case 8:
                
                break;
            case 9:
                
                break;
            case 10:
                
                break;
            case 11:
                
                break;
            case 12:
                
                break;
            case 13:
                currentMsg = "Test 13: Testing the scoring, (COPILOT) Press A for L4, B for L3, X for L2";
                currentStepPassed = false;
                break;
            case 14:
                currentMsg = "Test 14: Testing the centering, Press RB to center Right, LB to center Left, Y to center Center";
                break;
            case 15:
                
                break;

            default:
                robotController.ledsSystem.setState(LEDState.OFF); 
            break;
        }

        if (stepStarted) {
            System.out.println(currentMsg);
            stepStarted = true;
        }
        robotController.ledsSystem.setState(currentStepPassed ? LEDState.GREEN : LEDState.OFF);
        Logger.recordOutput("TestController/currentStep", testStep);
        Logger.recordOutput("TestController/currentStepPassed", currentStepPassed);
        Logger.recordOutput("TestController/currentMsg", currentMsg);
        robotController.ledsSystem.periodic();
     }
}
