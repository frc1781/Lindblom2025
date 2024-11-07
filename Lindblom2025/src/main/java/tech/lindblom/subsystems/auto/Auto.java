package tech.lindblom.subsystems.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.types.Subsystem;

public class Auto extends Subsystem {
    private LoggedDashboardChooser AutoChooser;
    private AutoRoutine currentAutoRoutine;
    private boolean pathsGeneratedForRed;
    private Timer autoTimer = new Timer();

    private int currentAutoStepIndex = 0;
    private AutoStep currentAutoStep;
    private AutoStep[] allAutoSteps;


    public Auto(AutoRoutine... routines) {
        super("AutoSystem");

        AutoChooser = new LoggedDashboardChooser<>("Auto Routine");
        for (AutoRoutine routine : routines) {
            AutoChooser.getSendableChooser().addOption(routine.getName(), routine);
        }
    }

    @Override
    public void init() {
        
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case DISABLED:
                checkSelectedRoutine();
                break;
            case AUTONOMOUS:

        }
    }

    public void checkSelectedRoutine() {
        boolean currentAlliance = RobotController.isRed();
        AutoRoutine chosenRoutine = (AutoRoutine) AutoChooser.getSendableChooser().getSelected();
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

    public class NoAutoRoutineException extends Exception {
        public NoAutoRoutineException() {
            super("No routine!");
        }

        @Override
        public void printStackTrace() {
            System.out.println("The routine was null or invalid. We will till we start auto.");
        }
    }
}
