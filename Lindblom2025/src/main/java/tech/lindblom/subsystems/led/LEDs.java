package tech.lindblom.subsystems.led;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import java.util.TreeMap;

import com.fasterxml.jackson.databind.ser.BeanSerializer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LEDs extends StateSubsystem {
    private final int LED_LENGTH = 150; // 75 + 75 = 150
    private RobotController robotController;

    private AddressableLED mLedController = null;
    private AddressableLEDBuffer mLedBuffer = null;
    private int mRainbowFirstPixelHue = 1;
    private Timer flashingTimer;
    private boolean flashAlt;

    public LEDs(RobotController _robotController) {
        super("LEDs", LEDState.RAINBOW);
        this.robotController = _robotController;
    }

    @Override
    public void init() {
        flashingTimer = new Timer();
        flashingTimer.reset();
        if (mLedController == null) {
            mLedController = new AddressableLED(1);
            mLedBuffer = new AddressableLEDBuffer(LED_LENGTH + 1);

            mLedController.setLength(mLedBuffer.getLength());
            mLedController.setData(mLedBuffer);
            mLedController.start();
        }

        switch (currentOperatingMode) {
            case DISABLED:  
                setState(LEDState.RAINBOW);
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        if ((mLedBuffer == null || mLedController == null)) {
            return;
        }

        switch((LEDState) getCurrentState()) {
            case RAINBOW:
                rainbow();
            case WHITE:
                solid(255, 255, 255);
                break;
            case RED:
                solid(255, 0, 0);
                break;
            case GREEN:
                solid(0, 255, 0);
                break;
            case BLUE:
                solid(0, 0, 255);
                break;
            case SYNC:
                flashing(0, 255, 0);
                if (timeInState.get() > 1) {
                    setState(LEDState.WHITE);
                }
                break;
            case OVER:
                flashing(255, 255, 0);
                break;
        }

        mLedController.setData(mLedBuffer);
    }

    private void solid(int r, int g, int b) {
        for(int i = 0; i < LED_LENGTH; i++) {
            mLedBuffer.setRGB(i, g,r,b);
        }
    }

    private void flashing(int r, int g, int b) {
        if (flashingTimer.get() > 0.2) {
            if (flashAlt) {
                solid(0, 0, 0);
            } else {
                solid(r, g, b);
            }
            flashingTimer.reset();
            flashingTimer.stop();
            flashAlt = !flashAlt;
        }

        flashingTimer.start();
    }

    private void rainbow() {
        for (var i = 0; i < mLedBuffer.getLength(); i++) {

            final var hue = (mRainbowFirstPixelHue + (i * 180 / mLedBuffer.getLength())) % 180;
            mLedBuffer.setHSV(i, hue, 255, 128);
        }

        mRainbowFirstPixelHue += 3;

        mRainbowFirstPixelHue %= 180;
    }


    @Override
    public boolean matchesState() {
        return this.getCurrentState() != LEDState.EXPECTED_FAIL;
    }

    public enum LEDState implements SubsystemState{
        RAINBOW, SYNC, WHITE, RED, GREEN, BLUE, OVER, EXPECTED_FAIL
    }
}
