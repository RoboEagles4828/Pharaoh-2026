
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double shooterOneVelocityRPS = 0.0;
        public double shooterTwoVelocityRPS = 0.0;
        public double shooterThreeVelocityRPS = 0.0;

        public double hoodPosition = 0.0;

        public double shooterOneCurrentAmps = 0.0;
        public double shooterTwoCurrentAmps = 0.0;
        public double shooterThreeCurrentAmps = 0.0;
        public double hoodCurrentAmps = 0.0;
    }

    // Read hardware inputs
    default void updateInputs(ShooterIOInputs inputs) {}

    // Configure motor PID and current limits
    default void configure(
        double shooterP,
        double shooterV,
        double hoodP,
        double hoodD
    ) {}

    // Shooter wheel control
    default void setShooterVelocity(
        double motorOneRPS,
        double motorTwoRPS,
        double motorThreeRPS
    ) {}

    default void stopShooter() {}

    // Hood control
    default void setHoodTargetPosition(double position) {}

    default void setHoodEncoderPosition(double position) {}
}