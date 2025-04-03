package tech.lindblom.subsystems.led;

import tech.lindblom.control.RobotController;
import tech.lindblom.subsystems.drive.DriveController;
import tech.lindblom.subsystems.drive.DriveController.DriverStates;
import tech.lindblom.subsystems.types.StateSubsystem;
import tech.lindblom.utils.EEUtil;
import tech.lindblom.utils.EnumCollection;
import tech.lindblom.utils.EnumCollection.OperatingMode;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class LEDs extends StateSubsystem {
    private final RobotController robotController;
    private final int LED_LENGTH = 150;

    private AddressableLED ledController = null;
    private AddressableLEDBuffer ledBuffer = null;

    private Timer flashingTimer;
    private Timer marchingTimer;

    public LEDs(RobotController _robotController) {
        super("LEDs", LEDState.OPERATING_COLOR);
        this.robotController = _robotController;

        ledController = new AddressableLED(1);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH + 1);
        ledController.setLength(ledBuffer.getLength());
        ledController.setData(ledBuffer);
        ledController.start();
    }

    @Override
    public void init() {
        super.init();

        flashingTimer = new Timer();
        flashingTimer.reset();

        marchingTimer = new Timer();
        marchingTimer.reset();
    }

    @Override
    public void periodic() {
        if ((ledBuffer == null || ledController == null)) {
            return;
        }

        switch((LEDState) getCurrentState()) {
            case OPERATING_COLOR:
                switch (currentOperatingMode) {
                    case AUTONOMOUS:
                        solid(250, 156, 28);
                        break;
                    case TELEOP:
                        solid(254, 254, 254);
                        break;
                    case DISABLED:
                        rainbow();
                        break;
                }
                break;
            case RAINBOW:
                rainbow();
                break;
            case MARCH_WHITE:
                marching(255, 255, 150);
                break;
            case WHITE:
                solid(0, 255, 255);
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
            case YELLOW:
                solid(255, 255, 0);
                break;
            case PURPLE:
                solid(255, 0, 255);
                break;
            case FLASH_YELLOW:
                flashing(255, 255, 0);
            case SYNC:
                flashing(0, 255, 0);
                if (timeInState.get() > 1) {
                    setState(LEDState.WHITE);
                }
                break;
            case OFF: 
                solid(0, 0, 0);
                break;
            case OVER:
                flashing(255, 255, 0);
                break;
            case EXPECTED_FAIL:
                flashing(255, 0, 0);
                break;
        }

        ledController.setData(ledBuffer);
    }

    private void solid(int r, int g, int b) {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, g, r, b);
        }
    }

    private boolean flash;

    private void flashing(int r, int g, int b) {
        if (flashingTimer.get() > 0.2) {
            if (flash) {
                solid(0, 0, 0);
            } else {
                solid(r, g, b);
            }
            flashingTimer.reset();
            flashingTimer.stop();
            flash = !flash;
        }

        flashingTimer.start();
    }

    private int rainbowFirstPixelHue = 1;

    private void rainbow() {
        for (int i = 0; i < LED_LENGTH; i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / LED_LENGTH)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    private int newMarcher;

    private void marching(int r, int g, int b) {
        if(marchingTimer.get() >= 0.03)
        {
            for(int i = LED_LENGTH - 1; i > 0; i--) {
                ledBuffer.setRGB(i, ledBuffer.getRed(i - 1), ledBuffer.getGreen(i - 1), ledBuffer.getBlue(i - 1));
            }

            if(newMarcher <= 0) {
                ledBuffer.setRGB(0, r, g, b);
                newMarcher = 4;
            }
            else
            {
                ledBuffer.setRGB(0, 0, 0, 0);
            }
            newMarcher--;

            marchingTimer.reset();
            marchingTimer.stop();
        }

        marchingTimer.start();
    }

    @Override
    public boolean matchesState() {
        return this.getCurrentState() != LEDState.EXPECTED_FAIL;
    }

    public enum LEDState implements SubsystemState {
        RAINBOW, 
        MARCH_WHITE,
        SYNC, 
        WHITE, 
        RED, 
        GREEN, 
        BLUE, 
        YELLOW, 
        FLASH_YELLOW, 
        PURPLE, 
        OVER, 
        OFF, 
        OPERATING_COLOR, 
        EXPECTED_FAIL
    }
}
