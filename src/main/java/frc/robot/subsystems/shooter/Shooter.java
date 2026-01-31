package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util4828;

/** The shooter subsystem controls the output of fuel. */
public class Shooter extends SubsystemBase {
    private static final TunableNumber shootingSpeedMPS = new TunableNumber(ShooterConstants.NT_TARGET_SPEED_MPS, ShooterConstants.DEFAULT_SPEED_MPS);
    
    /** Motor controlling the line of wheels */
    private final TalonFX motor;

    /** Request for controlling the motor in MPS */
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

    @Override
    public void periodic() {
        // output the current measured speed of the flywheel, for verification/tuning
        double actualMPS = motor.getVelocity().getValueAsDouble() / ShooterConstants.GEAR_RATIO * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualMPS);
    }

    /** Command to shoot the fuel. */
    public Command start() {
        return Commands.run(() -> {
            // convert from target meters per second to motor rotations per second and set on motor
            double motorRPS = Util4828.metersPerSecondToMotorRPS(shootingSpeedMPS.get(), ShooterConstants.WHEEL_DIAMETER, ShooterConstants.GEAR_RATIO);
            motor.setControl(velocityVoltageRequest.withVelocity(motorRPS));
        }, this);
    }
    
    /** Command to stop shooting fuel. */
    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }

}