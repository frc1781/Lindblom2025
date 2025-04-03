package tech.lindblom.utils;

import edu.wpi.first.wpilibj.Timer;

public class ControlCurve {
    
    private Timer curveTime;
    private double startPosition;
    private double endPosition;
    private double duration;

    public ControlCurve(double startPosition, double endPosition, double duration) {
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.duration = duration;
        curveTime.reset();
        curveTime.start();
    }

    public void startCurve() {
        curveTime.reset();
        curveTime.start();
    }

    public double calculate() {
        if (curveTime.get() > duration) {
            return endPosition;
        }

        return startPosition + Math.sin((curveTime.get()/duration) * Math.PI/2) * (endPosition - startPosition);
    }
}
