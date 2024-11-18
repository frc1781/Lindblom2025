package tech.lindblom.subsystems.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.subsystems.types.Subsystem;
import tech.lindblom.utils.Constants;
import tech.lindblom.utils.EnumCollection;

public class Auto extends Subsystem {
    private SendableChooser<AutoRoutine> autoChooser;
    private AutoRoutine currentAutoRoutine;
    private boolean pathsGeneratedForRed;
    private Timer autoTimer = new Timer();

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
            autoTimer.reset();
            currentAutoStepIndex = 0;
            checkSelectedRoutine();
            autoTimer.start();
        }
    }

    @Override
    public void periodic() throws RoutineOverExecption {
        switch (currentMode) {
            case DISABLED:
                checkSelectedRoutine();
                break;
            case AUTONOMOUS:
                boolean timeUp = currentAutoStep.hasTimeLimit() ? currentAutoStep.getMaxTime() < autoTimer.get() : false;
                boolean shouldEndRoutine = timeUp;

                if (currentAutoStepIndex == 0 || shouldEndRoutine) {
                    autoTimer.reset();

                    if (allAutoSteps.length == currentAutoStepIndex) {
                        throw new RoutineOverExecption();
                    }
                    currentAutoStep = allAutoSteps[currentAutoStepIndex];
                    startStep(currentAutoStep);
                    autoTimer.start();

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
            autoTimer.reset();
            currentAutoStepIndex = 0;
            currentAutoRoutine = chosenRoutine;
            allAutoSteps = currentAutoRoutine.getSteps();
            currentAutoStep = currentAutoRoutine.getSteps()[0];

            pathsGeneratedForRed = currentAlliance;
            Logger.recordOutput("Auto/Routine", currentAutoRoutine.getName());
            System.out.println("Cached currently selected routine");
        }
    }

    public Pose2d getStartPosition() throws NoAutoRoutineException {
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

        throw new NoAutoRoutineException();
    }

    public class RoutineOverExecption extends Exception {
        public RoutineOverExecption() {
            super("Routine Ended!");
        }

        @Override
        public void printStackTrace() {
            System.out.println("RoutineOverExecption: The Routine running was finished.");
        }
    }

    public class NoAutoRoutineException extends Exception {
        public NoAutoRoutineException() {
            super("No routine!");
        }

        @Override
        public void printStackTrace() {
            System.out.println("NoAutoRoutineException: The routine was null or invalid. Auto will not start.");
        }
    }
}
