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
    
     public TestController(RobotController robotController){
            private int testStep = 0;
            private boolean nextTestStepPushed = false;
            private boolean prevTestStepPushed = false;
        
 

     }

     public void periodic(){
           
                        boolean stepStarted = false;
                
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
                        
                        switch(testStep) {
                            case 0:
                                //do nothing
                                if (stepStarted) {
                                    System.out.println("Starting testing, hit B for first test");
                                }
                                ledsSystem.setState(LEDState.OFF);
                                break;
                            case 1:
                                if (stepStarted) {
                                    System.out.println("Test 1: testing leds, they should be red");
                                }
                                ledsSystem.setState(LEDState.RED);
                              break;
                            default:
                                ledsSystem.setState(LEDState.OFF); 
                            break;
                        }
                    

     }

}
