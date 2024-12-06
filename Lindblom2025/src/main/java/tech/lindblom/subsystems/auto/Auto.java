package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.auto.groups.AutoStepGroup;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Auto extends Subsystem {
    private SendableChooser<AutoRoutine> autoChooser;
    private AutoRoutine currentAutoRoutine;
    private boolean pathsGeneratedForRed;

    private int currentStepIndex = 0;
    private int currentGroupIndex = 0;

    private final RobotController robotController;

    private AutoStep currentStep;
    private AutoStep[] currentGroupSteps;

    private AutoStep[] reactionAutoSteps;
    private AutoStepGroup[] stepGroups;


    public Auto(RobotController robotController, AutoRoutine... routines) {
        super("Auto");

        autoChooser = new SendableChooser<>();
        for (AutoRoutine routine : routines) {
            autoChooser.addOption(routine.getName(), routine);
        }
        Constants.Auto.AUTONOMOUS_TAB.add(autoChooser);
        this.robotController = robotController;
    }

    @Override
    public void init() {
        if (currentMode == EnumCollection.OperatingMode.AUTONOMOUS) {
            robotController.autoTimer.reset();
            currentStepIndex = 0;
            checkSelectedRoutine();
            robotController.autoTimer.start();
        }
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case DISABLED:
                checkSelectedRoutine();
                break;
            case AUTONOMOUS:
                boolean timeUp = currentStep.getMaxTime() != 0 ? currentStep.getMaxTime() < robotController.autoTimer.get() : false;
                boolean finishedAutoStep = robotController.hasFinishedAutoStep();
                boolean shouldEndAutoStep = timeUp || finishedAutoStep;

                if (currentStepIndex == 0 || shouldEndAutoStep) {
                    robotController.autoTimer.reset();

                    if (stepGroups.length == currentGroupIndex) {
                        robotController.interruptAction();
                        return;
                    }

                    if (timeUp && stepGroups[currentGroupIndex].getGroupType() == AutoStepGroup.GroupType.DEPEND) {
                        //step failed
                    }

                    if (currentStepIndex == currentGroupSteps.length) {
                        currentStepIndex = 0;
                        currentGroupSteps = stepGroups[currentGroupIndex].getAutoSteps();
                        currentGroupIndex++;
                    }

                    currentStep = currentGroupSteps[currentStepIndex];
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
        Logger.recordOutput(name + "/CurrentAutoStepIndex", currentStepIndex);
        robotController.setAutoStep(step);
    }

/*    public void checkSelectedRoutine() {
        boolean currentAlliance = RobotController.isRed();
        AutoRoutine chosenRoutine = (AutoRoutine) autoChooser.getSelected();

        if (chosenRoutine == null) return;

        Logger.recordOutput(name + "/ChosenRoutine", chosenRoutine.getName());

        if (currentAutoRoutine != chosenRoutine || pathsGeneratedForRed != currentAlliance) {
            robotController.autoTimer.reset();
            currentAutoStepIndex = 0;
            currentAutoRoutine = chosenRoutine;
            allAutoSteps = currentAutoRoutine.getSteps();
            currentAutoStep = currentAutoRoutine.getSteps()[0];

            pathsGeneratedForRed = currentAlliance;
            System.out.println("Cached currently selected routine");
        }
    }*/

    public void checkSelectedRoutine() {
        boolean currentAlliance = RobotController.isRed();
        AutoRoutine chosenRoutine = autoChooser.getSelected();

        if (chosenRoutine == null) return;
        Logger.recordOutput(name + "/ChosenRoutine", chosenRoutine.getName());

        if (currentAutoRoutine != chosenRoutine || pathsGeneratedForRed != currentAlliance) {
            robotController.autoTimer.reset();
            currentStepIndex = 0;
            currentGroupIndex = 0;
            currentAutoRoutine = chosenRoutine;
            stepGroups = chosenRoutine.getAutoStepGroups();
            currentStep = stepGroups[0].getAutoSteps()[0];

            pathsGeneratedForRed = currentAlliance;
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
        var ret_val = PathPlannerPath.fromPathFile(name);
        ret_val.preventFlipping = false;
        if(RobotController.isRed()) {
            ret_val = ret_val.flipPath();
        }
        return ret_val;
    }

    public class NoStartingPositionException extends Exception {
        public NoStartingPositionException() {
            super("No starting position!");
        }

        @Override
        public void printStackTrace() {
            System.out.println("NoAutoRoutineException: This routine does not have a starting position, the position has been set to 0,0.");
        }
    }
}
