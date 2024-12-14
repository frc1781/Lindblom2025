package tech.lindblom.subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.Logger;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Auto extends Subsystem {
    private SendableChooser<AutoRoutine> autoChooser;
    private AutoRoutine currentAutoRoutine;
    private boolean pathsGeneratedForRed;

    private int currentAutoStepIndex = 0;
    private final RobotController robotController;
    private AutoStep currentAutoStep;
    private AutoStep[] allAutoSteps;


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
            currentAutoStepIndex = 0;
            checkSelectedRoutine();
            robotController.autoTimer.start();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.name + "/Arm Angle", 0);

        switch (currentMode) {
            case DISABLED:
                checkSelectedRoutine();
                break;
            case AUTONOMOUS:

                boolean timeUp = currentAutoStep.hasTimeLimit() ? currentAutoStep.getMaxTime() < robotController.autoTimer.get() : false;
                boolean shouldEndRoutine = timeUp;

                if (currentAutoStepIndex == 0 || shouldEndRoutine) {
                    robotController.autoTimer.reset();

                    if (allAutoSteps.length == currentAutoStepIndex) {
                        robotController.interruptAction();
                        return;
                    }

                    currentAutoStep = allAutoSteps[currentAutoStepIndex];
                    startStep(currentAutoStep);
                    robotController.autoTimer.start();

                    currentAutoStepIndex++;
                }
                break;
        }
    }

    private void startStep(AutoStep step) {
        Logger.recordOutput(name + "/CurrentAutoStepIndex", currentAutoStepIndex);
        robotController.setAutoStep(step);
    }

    public void checkSelectedRoutine() {
        boolean currentAlliance = RobotController.isRed();
        AutoRoutine chosenRoutine = (AutoRoutine) autoChooser.getSelected();

        if (chosenRoutine == null) return;

        Logger.recordOutput("Autonomous/ChosenRoutine", chosenRoutine.getName());

        if (currentAutoRoutine != chosenRoutine || pathsGeneratedForRed != currentAlliance) {
            robotController.autoTimer.reset();
            currentAutoStepIndex = 0;
            currentAutoRoutine = chosenRoutine;
            allAutoSteps = currentAutoRoutine.getSteps();
            currentAutoStep = currentAutoRoutine.getSteps()[0];

            pathsGeneratedForRed = currentAlliance;
            Logger.recordOutput("Auto/Routine", currentAutoRoutine.getName());
            System.out.println("Cached currently selected routine");
        }
    }

    public Pose2d getStartPosition() throws NoStartingPositionException {
        if (currentAutoRoutine != null) {
            for (int i = 0; i < allAutoSteps.length; i++) {
                switch (allAutoSteps[i].getStepType()) {
                    case PATH_AND_ACTION:
                    case PATH:
                        return allAutoSteps[i].getPath().getStartingDifferentialPose();
                }
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
