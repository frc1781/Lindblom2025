package tech.lindblom.subsystems.led;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.drive.DriveController.DriverStates;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LEDs extends StateSubsystem {
    private final RobotController robotController;
    private final int LED_LENGTH = 150;

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
    }

    @Override
    public void periodic() {
        if ((mLedBuffer == null || mLedController == null)) {
            return;
        }

        switch((LEDState) getCurrentState()) {
            case OPERATING_COLOR, RAINBOW:
                rainbow();
                break;
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
            case PURPLE:
                solid(255, 0, 255);
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
            mLedBuffer.setRGB(i, r, g, b);
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
        for (var i = 0; i < LED_LENGTH; i++) {
            var hue = (mRainbowFirstPixelHue + (i * 180 / LED_LENGTH)) % 180;
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
        RAINBOW, SYNC, WHITE, RED, GREEN, BLUE, PURPLE, OVER, OFF, OPERATING_COLOR, EXPECTED_FAIL
    }
}
