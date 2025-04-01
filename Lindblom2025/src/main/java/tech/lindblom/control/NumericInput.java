package tech.lindblom.control;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NumericInput {
    private double p;

    public NumericInput(RobotController rc) {
        SmartDashboard.putNumber("p", 0.01);
    }

    public void periodic() {
        p = SmartDashboard.getNumber("p", 0.0);
    }

    public double getNumber(String key, double defaultValue) {
        return SmartDashboard.getNumber(key, defaultValue);
    }
}
