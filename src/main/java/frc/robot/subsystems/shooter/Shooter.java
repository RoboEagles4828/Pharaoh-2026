package frc.robot.subsystems.shooter;

import java.util.Collections;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
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
    private static final TunableNumber hoodPosition = new TunableNumber(ShooterConstants.NT_TARGET_HOOD_POSITION, ShooterConstants.HOOD_STARTING_POSITION);
    private static final TunableNumber hoodPValue = new TunableNumber(ShooterConstants.NT_HOOD_P_VALUE, ShooterConstants.HOOD_PID_CONFIG.PROPORTIONAL);

    /** Motor controlling the launching wheels */
    //one
    private final TalonFX shooterMotorOne;
    //two
    private final TalonFX shooterMotorTwo;
    //three
    private final TalonFX shooterMotorThree;
    /** Motor controlling the kicker */
    private final TalonFX kickerMotor;

    /** Motor controlling the hood */
    private final TalonFX hoodMotor;

    /** Request for controlling the motor in MPS */
    private final VelocityVoltage shooterVelocityVoltageRequest = new VelocityVoltage(0);
    private final VelocityVoltage kickerVelocityVoltageRequest = new VelocityVoltage(0);
    private final PositionVoltage hoodPositionVoltageRequest = new PositionVoltage(ShooterConstants.HOOD_STARTING_POSITION)
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
                                        .withSlot(0);

    public Shooter() {
        shooterMotorOne = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_ONE_ID);
        shooterMotorTwo = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_TWO_ID);
        shooterMotorThree = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_THREE_ID);
        kickerMotor = new TalonFX(RioBusCANIds.KICKER_MOTOR_ID);
        hoodMotor = new TalonFX(RioBusCANIds.HOOD_MOTOR_ID);

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
        
        final TalonFXConfiguration hoodMotorCfg = new TalonFXConfiguration();
        hoodMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodMotorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;
        hoodMotorCfg.Slot0.kP = ShooterConstants.HOOD_PID_CONFIG.PROPORTIONAL;
        hoodMotorCfg.Slot0.kD = ShooterConstants.HOOD_PID_CONFIG.DERIVATIVE;

        // Applying the configuration
        shooterMotorOne.getConfigurator().apply(shooterMotorCfg);
        shooterMotorTwo.getConfigurator().apply(shooterMotorCfg);
        shooterMotorThree.getConfigurator().apply(shooterMotorCfg);
        kickerMotor.getConfigurator().apply(kickerMotorCfg);
        hoodMotor.getConfigurator().apply(hoodMotorCfg);

        SmartDashboard.putBoolean(ShooterConstants.NT_APPLY_PID_BUTTON, false);
    }

    /** Command to shoot the fuel. */
    public Command start() {
        return Commands.run(() -> {
            // convert from target meters per second to wheel rotations per second
            double shootingwheelRPS = Util4828.metersPerSecondToWheelRPS(shootingSpeedMPS.get(), ShooterConstants.WHEEL_DIAMETER);
            SmartDashboard.putNumber("Tuning/Shooter/Shooter RPS", shootingwheelRPS);
            shooterMotorOne.setControl(shooterVelocityVoltageRequest.withVelocity(shootingwheelRPS));
            shooterMotorTwo.setControl(shooterVelocityVoltageRequest.withVelocity(shootingwheelRPS));
            shooterMotorThree.setControl(shooterVelocityVoltageRequest.withVelocity(shootingwheelRPS));
            // convert from target meters per second to wheel rotations per second
            double kickerWheelRPS = Util4828.metersPerSecondToWheelRPS(kickingSpeedMPS.get(), ShooterConstants.KICKER_WHEEL_DIAMETER);
            SmartDashboard.putNumber("Tuning/Shooter/Kicker RPS", kickerWheelRPS);
            kickerMotor.setControl(kickerVelocityVoltageRequest.withVelocity(kickerWheelRPS));
        }, this);
    }
    
    /** Command to stop shooting fuel. */
    public Command stop() {
        return Commands.runOnce(() -> {
            shooterMotorOne.stopMotor();
            shooterMotorTwo.stopMotor();
            shooterMotorThree.stopMotor();
            kickerMotor.stopMotor();
        }, this);
    }

    public Command raiseHood() {
        return Commands.defer(
            () -> {
                return Commands.run(
                    () -> {
                        hoodMotor.setControl(hoodPositionVoltageRequest.withPosition(hoodPosition.get()));
                    }
                );
            },
            Collections.emptySet()
        );
    }

    public Command lowerHood() {
        return Commands.run(() -> hoodMotor.setControl(hoodPositionVoltageRequest.withPosition(ShooterConstants.HOOD_STARTING_POSITION)));
    }

    @Override
    public void periodic() {
        // apply new PID configs 
        if (SmartDashboard.getBoolean(ShooterConstants.NT_APPLY_PID_BUTTON, false)) {
            // TODO applyPIDConfigs();

            SmartDashboard.putBoolean(ShooterConstants.NT_APPLY_PID_BUTTON, false); // reset btn
        }

        // output the current measured speed of the flywheel, for verification/tuning
        double actualMPS_ONE = shooterMotorOne.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualMPS_ONE);
        double actualMPS_TWO = shooterMotorTwo.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualMPS_TWO);
        double actualMPS_THREE = shooterMotorThree.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualMPS_THREE);
        double actualKickerMPS = kickerMotor.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.KICKER_WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_KICKER_SPEED_MPS, actualKickerMPS);

        // output actual hood position
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_HOOD_POSITION, hoodMotor.getPosition().getValueAsDouble());
    }

}