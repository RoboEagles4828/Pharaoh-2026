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
    private static final TunableNumber kickingSpeedMPS = new TunableNumber(ShooterConstants.NT_KICKER_TARGET_SPEED_MPS, ShooterConstants.DEFAULT_KICKER_SPEED_MPS);
    
    
    /** Motor controlling the launching wheels */
    private final TalonFX shooterMotor;
    /** Motor controlling the kicker */
    private final TalonFX kickerMotor;

    /** Request for controlling the motor in MPS */
    private final VelocityVoltage shooterVelocityVoltageRequest = new VelocityVoltage(0);
    private final VelocityVoltage kickerVelocityVoltageRequest = new VelocityVoltage(0);

    public Shooter() {
        shooterMotor = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_ID);
        kickerMotor = new TalonFX(RioBusCANIds.KICKER_MOTOR_ID);

        /** Used to configure motors and PID slots */
        final TalonFXConfiguration shooterMotorCfg = new TalonFXConfiguration();
        shooterMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;
        shooterMotorCfg.Slot0.kS = ShooterConstants.PID_CONFIG.STATIC;
        shooterMotorCfg.Slot0.kV = ShooterConstants.PID_CONFIG.VELOCITY;
        shooterMotorCfg.Slot0.kP = ShooterConstants.PID_CONFIG.PROPORTIONAL;
        shooterMotorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        shooterMotorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;

        final TalonFXConfiguration kickerMotorCfg = new TalonFXConfiguration();
        kickerMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerMotorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.KICKER_GEAR_RATIO;
        kickerMotorCfg.Slot0.kS = ShooterConstants.PID_CONFIG.STATIC;
        kickerMotorCfg.Slot0.kV = ShooterConstants.PID_CONFIG.VELOCITY;
        kickerMotorCfg.Slot0.kP = ShooterConstants.PID_CONFIG.PROPORTIONAL;
        kickerMotorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        kickerMotorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;
        
        // Applying the configuration
        shooterMotor.getConfigurator().apply(shooterMotorCfg);
        kickerMotor.getConfigurator().apply(kickerMotorCfg);
    }

    /** Command to shoot the fuel. */
    public Command start() {
        return Commands.run(() -> {
            // convert from target meters per second to wheel rotations per second
            double shootingwheelRPS = ShooterConstants.SHOOTER_GEAR_RATIO * Util4828.metersPerSecondToWheelRPS(shootingSpeedMPS.get(), ShooterConstants.WHEEL_DIAMETER);
            SmartDashboard.putNumber("Tuning/Shooter/Shooter RPS", shootingwheelRPS);
            shooterMotor.setControl(shooterVelocityVoltageRequest.withVelocity(shootingwheelRPS));
            // convert from target meters per second to wheel rotations per second
            double kickerWheelRPS = ShooterConstants.KICKER_GEAR_RATIO * Util4828.metersPerSecondToWheelRPS(kickingSpeedMPS.get(), ShooterConstants.KICKER_WHEEL_DIAMETER);
            SmartDashboard.putNumber("Tuning/Shooter/Kicker RPS", kickerWheelRPS);
            kickerMotor.setControl(kickerVelocityVoltageRequest.withVelocity(kickerWheelRPS));
        }, this);
    }
    
    /** Command to stop shooting fuel. */
    public Command stop() {
        return Commands.runOnce(() -> {
            shooterMotor.stopMotor();
            kickerMotor.stopMotor();
        }, this);
    }

    @Override
    public void periodic() {
        // output the current measured speed of the flywheel, for verification/tuning
        double actualMPS = shooterMotor.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualMPS);
        double actualKickerMPS = kickerMotor.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.KICKER_WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_KICKER_SPEED_MPS, actualKickerMPS);
    }

}