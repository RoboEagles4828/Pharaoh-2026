package frc.robot.subsystems.shooter;

public class ShooterParameters {
    private double distance; // meters
    private double flywheelSpeed; // meters per second
    private double hoodValue; // 0-1 for rack&pinion extension

    public ShooterParameters(double d, double f, double h) {
        distance = d;
        flywheelSpeed = f;
        hoodValue = h;
    }

    public double getDistance() {
        return distance;
    }
    
    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }

    public double getHoodValue() {
        return hoodValue;
    }
}