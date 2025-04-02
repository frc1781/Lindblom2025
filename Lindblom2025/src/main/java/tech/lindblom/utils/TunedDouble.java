package tech.lindblom.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.DoubleSupplier;

public class TunedDouble implements DoubleSupplier {
    private static final String tableKey = "/Tuning";
    private String key;

    private Double defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private int lastHash;

    public TunedDouble(String key, Double defaultValue) {
        this.defaultValue = defaultValue;
        this.lastHash = this.defaultValue.hashCode();
        this.key = tableKey + "/" + key;
    }

    public boolean hasChanged() {
        Double currentValue = getAsDouble();

        return currentValue.hashCode() != lastHash;
    }


    @Override
    public double getAsDouble() {
        return Constants.General.DEBUG_MODE ? dashboardNumber.get() : defaultValue;
    }
}
