package tech.lindblom.utils;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.Timer;

//Problem with reporting invalid times for one cycle even though really valid
public class EEtimeOfFlight {
    private Timer timeTOFInvalid;
    public TimeOfFlight tof;
    public EEtimeOfFlight(int id, double frequency) {
      tof = new TimeOfFlight(id);
      tof.setRangingMode(TimeOfFlight.RangingMode.Short, frequency);
      timeTOFInvalid = new Timer();
    }

    public double getRange() {
        return tof.getRange();
    }

    // public boolean isRangeValid() {
    //     return tof.isRangeValid();
    // }

    public boolean isRangeValidRegularCheck() {
        if (tof.isRangeValid()) {
            timeTOFInvalid.reset();
        }
        else {
            if (!timeTOFInvalid.isRunning()) {
                timeTOFInvalid.start();
            }
        }

        return !(timeTOFInvalid.get() > 0.1);  //only false if have been getting invalid times for more than 100 milliseconds
    }
}
