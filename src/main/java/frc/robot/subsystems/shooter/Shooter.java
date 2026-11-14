
package frc.robot.subsystems.shooter;

import java.util.Collections;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util4828;

public class Shooter extends SubsystemBase {

    private static final TunableNumber shootingSpeedMPS =
        new TunableNumber(
            "Tuning/Shooter/TargetShooterSpeedMPS",
            ShooterConstants.DEFAULT_SPEED_MPS
        );

    private static final TunableNumber intakeSpeedMPS =
        new TunableNumber("Tuning/Shooter/TargetIntakeSpeed", -2.0);

    private static final TunableNumber shooterPValue =
        new TunableNumber(
            "Tuning/Shooter/ShooterPValue",
            ShooterConstants.PID_CONFIG.PROPORTIONAL
        );

    private static final TunableNumber shooterVValue =
        new TunableNumber(
            "Tuning/Shooter/ShooterVValue",
            ShooterConstants.PID_CONFIG.VELOCITY
        );

    private static final TunableNumber hoodPosition =
        new TunableNumber(
            "Tuning/Shooter/TargetHoodPosition",
            ShooterConstants.HOOD_MIN_POSITION
        );

    private static final TunableNumber hoodPValue =
        new TunableNumber(
            "Tuning/Shooter/HoodPValue",
            ShooterConstants.HOOD_PID_CONFIG.PROPORTIONAL
        );

    private static final TunableNumber hoodDValue =
        new TunableNumber(
            "Tuning/Shooter/HoodDValue",
            ShooterConstants.HOOD_PID_CONFIG.DERIVATIVE
        );

    // Hardware abstraction
    private final ShooterIO io;

    // AdvantageKit logged inputs
    private final ShooterIOInputsAutoLogged inputs =
        new ShooterIOInputsAutoLogged();

    private final LaunchCalculator launchCalculator;

    private double targetHoodPosition = 0.0;
    private double targetLaunchVelocity = 0.0;

    public Shooter(ShooterIO io, LaunchCalculator launchCalculator) {
        this.io = io;
        this.launchCalculator = launchCalculator;

        updatePIDConfigs();

        // Initialize hood encoder
        io.setHoodEncoderPosition(
            ShooterConstants.HOOD_MIN_POSITION
        );

        if (Constants.debugMode) {
            SmartDashboard.putBoolean(
                ShooterConstants.NT_APPLY_PID_BUTTON,
                false
            );
        }
    }

    private void updatePIDConfigs() {
        io.configure(
            shooterPValue.get(),
            shooterVValue.get(),
            hoodPValue.get(),
            hoodDValue.get()
        );
    }

    /** Returns a command to shoot the fuel. */
    public Command start() {
        return Commands.run(() -> {
            double shootingWheelRPS =
                Util4828.metersPerSecondToWheelRPS(
                    targetLaunchVelocity,
                    ShooterConstants.WHEEL_DIAMETER
                );

            io.setShooterVelocity(
                -shootingWheelRPS,
                shootingWheelRPS,
                shootingWheelRPS
            );
        }, this);
    }

    /** Returns a command that spins the shooter wheels during intake. */
    public Command startIntake() {
        return Commands.run(() -> {
            // Your original intake implementation was commented out.
            // Add the desired IO command here when ready.
        }, this);
    }

    /** Returns a command to stop the shooting motors. */
    public Command stop() {
        return Commands.runOnce(() -> {
            io.stopShooter();
        }, this);
    }

    /** Returns a command that raises the hood. */
    public Command raiseHood() {
        return Commands.run(() -> {
            double position = MathUtil.clamp(
                targetHoodPosition,
                ShooterConstants.HOOD_MIN_POSITION,
                ShooterConstants.HOOD_MAX_POSITION
            );

            io.setHoodTargetPosition(position);
        }, this);
    }

    /** Returns a command that lowers the hood. */
    public Command lowerHood() {
        return Commands.runOnce(() -> {
            io.setHoodTargetPosition(
                ShooterConstants.HOOD_MIN_POSITION
            );
        }, this);
    }

    /** Resets the hood encoder. */
    public Command resetHoodEncoder() {
        return Commands.runOnce(() -> {
            io.setHoodEncoderPosition(
                ShooterConstants.HOOD_MIN_POSITION
            );
        }, this);
    }

    private void setTargetParams(
        double targetLaunchVelocity,
        double targetHoodPosition
    ) {
        this.targetLaunchVelocity = targetLaunchVelocity;
        this.targetHoodPosition = targetHoodPosition;
    }

    /** Returns whether the shooter is at its target parameters. */
    public boolean isAtTargetParams() {

        double actualMPS_ONE =
            inputs.shooterOneVelocityRPS
                * Math.PI * ShooterConstants.WHEEL_DIAMETER;

        double actualMPS_TWO =
            inputs.shooterTwoVelocityRPS
                * Math.PI * ShooterConstants.WHEEL_DIAMETER;

        double actualMPS_THREE =
            inputs.shooterThreeVelocityRPS
                * Math.PI * ShooterConstants.WHEEL_DIAMETER;

        boolean hoodReady =
            Math.abs(inputs.hoodPosition - targetHoodPosition)
                < ShooterConstants.HOOD_POSITION_TOLERANCE;

        boolean shooterOneReady =
            Math.abs(actualMPS_ONE - targetLaunchVelocity)
                < ShooterConstants.SHOOTER_SPEED_TOLERANCE;

        boolean shooterTwoReady =
            Math.abs(actualMPS_TWO - targetLaunchVelocity)
                < ShooterConstants.SHOOTER_SPEED_TOLERANCE;

        boolean shooterThreeReady =
            Math.abs(actualMPS_THREE - targetLaunchVelocity)
                < ShooterConstants.SHOOTER_SPEED_TOLERANCE;

        return hoodReady &&
            (shooterOneReady || shooterTwoReady || shooterThreeReady);
    }

    @Override
    public void periodic() {

        // 1. Read hardware inputs
        io.updateInputs(inputs);

        // 2. Log inputs (and replace with replay inputs when replaying)
        Logger.processInputs("Shooter", inputs);

        // 3. Update target parameters
        var launchParams = launchCalculator.getParameters();

        setTargetParams(
            launchParams.velocityMPS(),
            launchParams.hoodPosition()
        );

        // 4. Apply updated PID configurations when requested
        if (Constants.debugMode) {
            if (SmartDashboard.getBoolean(
                ShooterConstants.NT_APPLY_PID_BUTTON,
                false
            )) {
                updatePIDConfigs();

                SmartDashboard.putBoolean(
                    ShooterConstants.NT_APPLY_PID_BUTTON,
                    false
                );
            }
        }

        // 5. Log calculated values and subsystem state
        double actualMPS_ONE =
            inputs.shooterOneVelocityRPS
                * Math.PI * ShooterConstants.WHEEL_DIAMETER;

        double actualMPS_TWO =
            inputs.shooterTwoVelocityRPS
                * Math.PI * ShooterConstants.WHEEL_DIAMETER;

        double actualMPS_THREE =
            inputs.shooterThreeVelocityRPS
                * Math.PI * ShooterConstants.WHEEL_DIAMETER;

        Logger.recordOutput(
            "Shooter/TargetVelocityMPS",
            targetLaunchVelocity
        );

        Logger.recordOutput(
            "Shooter/TargetHoodPosition",
            targetHoodPosition
        );

        Logger.recordOutput(
            "Shooter/ActualVelocityMPS/One",
            actualMPS_ONE
        );

        Logger.recordOutput(
            "Shooter/ActualVelocityMPS/Two",
            actualMPS_TWO
        );

        Logger.recordOutput(
            "Shooter/ActualVelocityMPS/Three",
            actualMPS_THREE
        );

        Logger.recordOutput(
            "Shooter/ActualHoodPosition",
            inputs.hoodPosition
        );

        Logger.recordOutput(
            "Shooter/Ready",
            isAtTargetParams()
        );

        Logger.recordOutput(
            "Shooter/Mode",
            launchCalculator.getMode()
        );
    }
}
