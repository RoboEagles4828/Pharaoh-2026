package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;

/** The shooter subsystem controls the output of fuel. */
public class Shooter extends SubsystemBase {
    private static final TunableNumber shootingSpeedRPS = new TunableNumber(ShooterConstants.NT_SHOOTING_SPEED_KEY, ShooterConstants.DEFAULT_SHOOTING_SPEED_RPS);
    /** Motor controlling the line of wheels */
    private final TalonFX motor;

    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    public Shooter() {
        motor = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_ID);

        /** Used to configure motors and PID slots */
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorCfg.Slot0.kP = ShooterConstants.PID_CONFIG.PROPORTIONAL;
        motorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        motorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;
        motorCfg.Slot0.kV = ShooterConstants.PID_CONFIG.VELOCITY;

        // Applying the configuration
        motor.getConfigurator().apply(motorCfg);
    }

    /** Command to shoot the fuel. */
    public Command start() {
        return Commands.run(() -> {
            motor.setControl(velocityVoltageRequest.withVelocity(shootingSpeedRPS.get()));
        }, this);
    }
    
    /** Command to stop shooting fuel. */
    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }

}