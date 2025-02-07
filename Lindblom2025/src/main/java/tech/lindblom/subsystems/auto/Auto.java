package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.routines.TestRoutine;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

import java.nio.file.Path;

public class Auto extends Subsystem {
    private SendableChooser<AutoRoutine> autoChooser;
    private AutoRoutine currentAutoRoutine;

    private int currentStepIndex = 0;
    private int currentGroupIndex = 0;
    private int currentReactionIndex = 0;

    private final RobotController robotController;

    private AutoStep currentStep;
    private AutoStep[] currentSteps;

    private AutoStep[] reactionSteps;
    private AutoStepGroup[] stepGroups;

    private AutoRoutine testRoutine;

    private boolean isReacting = false;


    public Auto(RobotController robotController, AutoRoutine... routines) {
        super("Auto");

        autoChooser = new SendableChooser<>();
        for (AutoRoutine routine : routines) {
            autoChooser.addOption(routine.getName(), routine);
        }
        Constants.Auto.AUTONOMOUS_TAB.add(autoChooser);
        this.robotController = robotController;

        testRoutine = new TestRoutine();
    }

    @Override
    public void init() {
        if (currentMode == EnumCollection.OperatingMode.AUTONOMOUS) {
            robotController.autoTimer.reset();
            currentStepIndex = 0;
            checkSelectedRoutine();
            robotController.autoTimer.start();

            currentGroupIndex = 0;
            currentReactionIndex = 0;
            currentStepIndex = 0;

            currentSteps = stepGroups[currentGroupIndex].getAutoSteps();
            currentGroupIndex++;
        }
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case DISABLED:
                checkSelectedRoutine();
                break;
            case AUTONOMOUS:
                boolean timeUp = currentStep.getMaxTime() != 0 && currentStep.getMaxTime() < robotController.autoTimer.get();
                boolean finishedAutoStep = robotController.hasFinishedAutoStep();
                boolean shouldEndAutoStep = timeUp || finishedAutoStep;

                Logger.recordOutput(name + "/CurrentGroupIndex", currentGroupIndex);
                Logger.recordOutput(name + "/CurrentStepIndex", currentStepIndex);
                Logger.recordOutput(name + "/CurrentReactionIndex", currentReactionIndex);

                Logger.recordOutput(name + "/CurrentAction", currentStep.getAction());

                if (currentStepIndex == 0 || shouldEndAutoStep) {
                    robotController.autoTimer.reset();

                    if (timeUp && stepGroups[currentGroupIndex].getGroupType() == AutoStepGroup.GroupType.DEPEND) {
                        if (currentStep.hasReaction()) {
                            reactionSteps = currentStep.getReaction().getReaction(robotController.getFailedSubsystems());
                            currentReactionIndex = 0;
                            currentStep = reactionSteps[currentReactionIndex];
                            currentReactionIndex++;
                            isReacting = true;

                            startStep(currentStep);
                            robotController.autoTimer.start();
                            return;
                        } else {
                            currentStepIndex = 0;
                            currentSteps = stepGroups[currentGroupIndex].getAutoSteps();
                            currentGroupIndex++;
                        }
                    }

                    if (isReacting) {
                        if (currentReactionIndex == reactionSteps.length) {
                            currentSteps = stepGroups[currentGroupIndex - 1].getAutoSteps();
                            currentStep = currentSteps[currentStepIndex - 1];
                            currentStepIndex++;
                            isReacting = false;
                            startStep(currentStep);
                            robotController.autoTimer.start();
                            return;
                        }

                        currentStep = reactionSteps[currentReactionIndex];
                        currentStepIndex++;
                        startStep(currentStep);
                        robotController.autoTimer.start();
                    }

                    if (currentStepIndex == currentSteps.length) {
                        if (stepGroups.length == currentGroupIndex) {
                            robotController.interruptAction();
                            return;
                        }

                        currentStepIndex = 0;
                        currentSteps = stepGroups[currentGroupIndex].getAutoSteps();
                        currentGroupIndex++;
                    }

                    currentStep = currentSteps[currentStepIndex];
                    startStep(currentStep);
                    robotController.autoTimer.start();

                    currentStepIndex++;
                }
                break;
        }
    }

    public AutoStep getCurrentStep() {
        return this.currentStep;
    }

    private void startStep(AutoStep step) {
        robotController.setAutoStep(step);
    }

    public void checkSelectedRoutine() {
        // boolean currentAlliance = true; //TESTING TEMPORARY RobotController.isRed();
        AutoRoutine chosenRoutine = testRoutine; //autoChooser.getSelected();

        if (chosenRoutine == null) return;
        Logger.recordOutput(name + "/ChosenRoutine", chosenRoutine.getName());

        if (currentAutoRoutine == null || !currentAutoRoutine.getName().equals(chosenRoutine.getName())) {
            robotController.autoTimer.reset();
            currentStepIndex = 0;
            currentGroupIndex = 0;
            currentAutoRoutine = chosenRoutine;
            stepGroups = chosenRoutine.getAutoStepGroups();
            currentStep = stepGroups[0].getAutoSteps()[0];

           
            System.out.println("Cached current auto routine: " + currentAutoRoutine.getName());
        }
    }

    public Pose2d getStartPosition() throws NoStartingPositionException {
        if (currentAutoRoutine != null && currentStepIndex == 0) {
            switch (currentStep.getStepType()) {
                case PATH_AND_ACTION:
                case PATH:
                    return currentStep.getPath().getStartingDifferentialPose();
            }
        } else {
            System.out.println("Selected routine is null");
        }

        throw new NoStartingPositionException();
    }

    public static PathPlannerPath getPathFromName(String name) {
        try {
            var ret_val = PathPlannerPath.fromPathFile(name);
            ret_val.preventFlipping = false;
            if(RobotController.isRed()) {
                ret_val = ret_val.flipPath();
                System.out.println("5dd555555555555555555555555555555555555555555555555");
            }
            return ret_val;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public class NoStartingPositionException extends Exception {
        public NoStartingPositionException() {
            super("No starting position!");
        }

        @Override
        public void printStackTrace() {
            System.out.println("NoAutoRoutineException: This routine does not have a starting position, the position has been set to 0,0. We will not be moving in this AUTO period.");
        }
    }
}
