package tech.lindblom.subsystems.types;

import tech.lindblom.utils.EnumCollection;

public abstract class Subsystem {
    public final String name;
    protected EnumCollection.OperatingMode currentOperatingMode;

    protected Subsystem(String name) {
        this.name = name;
    }

    public void setOperatingMode(EnumCollection.OperatingMode mode) {
        currentOperatingMode = mode;
        System.out.println(name + " initialized into operating mode " + mode.toString());
        init();
    }

    public abstract void init();

    public abstract void periodic();
}
