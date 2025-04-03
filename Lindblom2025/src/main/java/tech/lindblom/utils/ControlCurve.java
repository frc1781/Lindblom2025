package tech.lindblom.utils;

import edu.wpi.first.wpilibj.Timer;

public class ControlCurve {
    
    private Timer curveTime;
    private double startPosition;
    private double endPosition;
    private double duration;
    private boolean started;

    public ControlCurve(double startPosition, double endPosition, double duration) {
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.duration = duration;
        curveTime = new Timer();
        started = false;
    }

    public double calculate() {
        if (!started) {
            curveTime.restart();
            started = true;
        }
        if (curveTime.get() > duration) {
            return endPosition;
        }

        return 
            startPosition + 
                (Math.sin(Math.PI * curveTime.get()/duration - Math.PI/2.0) + 1.0) *
                    0.5 * (endPosition - startPosition);
    }
}
