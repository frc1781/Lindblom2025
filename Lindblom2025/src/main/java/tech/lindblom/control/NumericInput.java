package tech.lindblom.control;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NumericInput {
    private double kP, dur, tar;

    public NumericInput(RobotController rc) {
        SmartDashboard.putNumber("kP", 0.01);
        SmartDashboard.putNumber("dur", 4.0);
        SmartDashboard.putNumber("tar", 60.0);
    }

    public void periodic() {
        kP = SmartDashboard.getNumber("kP", 0.01);
        dur = SmartDashboard.getNumber("dur", 4.0);
        tar = SmartDashboard.getNumber("tar", 60.0);
    }

    public double getNumber(String key, double defaultValue) {
        return SmartDashboard.getNumber(key, defaultValue);
    }
}
