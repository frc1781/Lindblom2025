// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/LICENSE
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
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public TunedDouble(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
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
            return 0.0;
        } else {
            return Constants.General.DEBUG_MODE ? dashboardNumber.get() : defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
