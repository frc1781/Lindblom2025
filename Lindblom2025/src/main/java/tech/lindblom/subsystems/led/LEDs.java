package tech.lindblom.subsystems.led;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.EnumCollection.OperatingMode;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends StateSubsystem {
    private final int LED_LENGTH = 75 + 75;
    private RobotController robotController;

    private AddressableLED mLedController = null;
    private AddressableLEDBuffer mLedBuffer = null;

    public LEDs() {
        super("LEDs", LEDState.OPERATING_COLOR);
    }


    @Override
    public void init() {
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

        switch ((LEDState) getCurrentState()) {
            case OPERATING_COLOR:
                if (currentOperatingMode == OperatingMode.TELEOP) {
                    solid(128, 0, 0);                                         
                } else if (currentOperatingMode == OperatingMode.AUTONOMOUS) {
                    solid(255, 215, 0);       
                }
                if (robotController.driveController.hasFoundReefPole()) {
                    solid(0, 255, 0);
                }
                break;
            case RED:
                solid(255,0,0);
                break;
            case WHITE:
                solid(255, 255, 255);
                break;
            case GREEN:
                solid(0,255,0);
                break;
            case BLUE:
                solid(0, 0, 255);
                break;
            case EXPECTED_FAIL:
                break;
        }
        mLedController.setData(mLedBuffer);
    }

    private void solid(int r, int g, int b) {
        for(int i = 0; i < LED_LENGTH; i++) {
            mLedBuffer.setRGB(i, g,r,b);
        }
    }

    @Override
    public boolean matchesState() {
        return this.getCurrentState() != LEDState.EXPECTED_FAIL;
    }

    public enum LEDState implements SubsystemState{
        OPERATING_COLOR, WHITE, RED, GREEN, BLUE, EXPECTED_FAIL
    }
}
