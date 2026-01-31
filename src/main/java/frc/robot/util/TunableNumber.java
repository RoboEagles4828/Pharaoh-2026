package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private String key;
    private double defaultValue;
    private double value;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        SmartDashboard.putNumber(key, defaultValue);
    }

    public double get() {
        value = SmartDashboard.getNumber(key, defaultValue);
        return value;
    }

    public String getKey() {
        return key;
    }

    public double getDefaultValue() {
        return defaultValue;
    }
}