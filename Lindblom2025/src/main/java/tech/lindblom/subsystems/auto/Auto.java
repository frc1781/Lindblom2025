package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.auto.routines.CenterOneCoral;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Auto extends Subsystem {
    private SendableChooser<AutoRoutine> autoChooser;
    private AutoRoutine currentAutoRoutine;
    private boolean isRedWhenBuilt;
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

        testRoutine = new CenterOneCoral();
    }

    @Override
    public void init() {
        if (currentOperatingMode == EnumCollection.OperatingMode.AUTONOMOUS) {
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
        switch (currentOperatingMode) {
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
                Logger.recordOutput(name + "/CurrentTime", robotController.autoTimer.get());

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
        robotController.interruptAction();
        robotController.setAutoStep(step);
    }

    public void checkSelectedRoutine() {
        // boolean currentAlliance = true; //TESTING TEMPORARY RobotController.isRed();
        AutoRoutine chosenRoutine = autoChooser.getSelected(); //testRoutine;

        if (chosenRoutine == null) {
            chosenRoutine = new CenterOneCoral();
        }
        
        Logger.recordOutput(name + "/ChosenRoutine", chosenRoutine.getName());

        if (currentAutoRoutine == null || 
          !currentAutoRoutine.getName().equals(chosenRoutine.getName()) ||
          RobotController.isRed() != isRedWhenBuilt
        ) {
            isRedWhenBuilt = RobotController.isRed();
            robotController.autoTimer.reset();
            currentStepIndex = 0;
            currentGroupIndex = 0;
            currentAutoRoutine = chosenRoutine;
            stepGroups = chosenRoutine.getAutoStepGroups(); //Reloads paths
            currentStep = stepGroups[0].getAutoSteps()[0];
            System.out.println("Cached current auto routine: " + currentAutoRoutine.getName());
        }
    }

    public Pose2d getStartPosition() throws NoStartingPositionException {
        if (currentAutoRoutine != null) {
            switch (currentStep.getStepType()) {
                case PATH_AND_ACTION:
                case PATH:
                    return new Pose2d(
                        currentStep.getPath().getStartingDifferentialPose().getTranslation(),
                        currentStep.getPath().getIdealStartingState().rotation());
            }
        } else {
            System.out.println("Selected routine is null");
        }

        throw new NoStartingPositionException();
    }
    
    public static PathPlannerPath getPathFromName(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
           //MODIFIED CHECK ret_val.preventFlipping = false;
           System.out.println("===========AUTO PATH LOADED=========================");
            if(RobotController.isRed()) {
                path = path.flipPath();
                System.out.println("FLIPPED PATH BECAUSE WE ARE ON RED ALLIANCE SIDE");
            }
            System.out.println("Loaded path " + name);
            System.out.println("====================================================");
            return path;
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
