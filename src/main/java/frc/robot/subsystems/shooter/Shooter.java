package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;

/** The shooter subsystem controls the output of fuel. */
public class Shooter extends SubsystemBase {
    private static final String NT_SPEED_RPS = "Tuning/Shooter/SpeedRPS";

    /** Motor controlling the line of wheels */
    private static TalonFX motor;

    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    public Shooter() {
        motor = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_ID);

        /** Used to configure motors and PID slots. */
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorCfg.Slot0.kP = ShooterConstants.PID_CONFIG.PROPORTIONAL;
        motorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        motorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;
        motorCfg.Slot0.kV = ShooterConstants.PID_CONFIG.VELOCITY;

        /** Applying the configuration */
        motor.getConfigurator().apply(motorCfg);

        SmartDashboard.putNumber(NT_SPEED_RPS, ShooterConstants.DEFAULT_SPEED_RPS);
    }

    /**
     * Sets both motors to the speed provided.
     * 
     * @param speed
     */
    private void setControl(ControlRequest request) {
        motor.setControl(request);
    }
    /**
     * Sets the speed of both motors to 0.
     */
    private void stopMotors() {
        motor.set(0);
    }

    /** Command to shoot the fuel. */
    public Command shoot() {
        return Commands.runOnce(() -> {
            this.setControl(velocityVoltageRequest.withVelocity(SmartDashboard.getNumber(NT_SPEED_RPS, ShooterConstants.DEFAULT_SPEED_RPS)));
        });
    }
    
    /** Command to stop shooting fuel. */
    public Command stop() {
        return Commands.runOnce(() -> this.stopMotors());
    }

}