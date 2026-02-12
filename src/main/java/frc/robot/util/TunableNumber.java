package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private String key;
    private double defaultValue;
    private double value;

    /** 
     * Constructs a new tunable number object.  
     * <p>
     * This allows the user to change a constant value through SmartDashboard without having to recompile the code.
     * 
     * @param key The key (name) to use for SmartDashboard
     * @param defaultValue The default value to use if no value is set on SmartDashboard
     */
    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        SmartDashboard.putNumber(key, defaultValue);
    }

    /** Returns the current value of the tunable number, whether it is the default value or a value set on SmartDashboard. */
    public double get() {
        value = SmartDashboard.getNumber(key, defaultValue);
        return value;
    }

    /** Returns the key (name) of the tunable number. */
    public String getKey() {
        return key;
    }

    /** Returns the default value of the tunable number. */
    public double getDefaultValue() {
        return defaultValue;
    }

    /** Sets the default value of the tunable number. */
    public void setDefaultValue(double newDefaultValue) {
        this.defaultValue = newDefaultValue;
        SmartDashboard.putNumber(key, defaultValue);
    }
}