package tech.lindblom.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class TunedDouble implements DoubleSupplier {
    private static final String tableKey = "/Tuning";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private double previousValue;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public TunedDouble(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
        this.previousValue = 0.0;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunedDouble(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            this.previousValue = defaultValue;
            if (Constants.General.DEBUG_MODE) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return Constants.General.DEBUG_MODE ? dashboardNumber.get() : 0.0;
        } else {
            return Constants.General.DEBUG_MODE ? dashboardNumber.get() : defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged() {
        double currentValue = get();
        double lastValue = this.previousValue;

        if (currentValue != lastValue) {
            this.previousValue = currentValue;
            return true;
        }

        return false;
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
