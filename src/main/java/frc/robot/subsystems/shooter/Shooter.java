package frc.robot.subsystems.shooter;

import java.util.Collections;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DigitalIDS;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util4828;

/** The shooter subsystem controls the output of fuel. */
public class Shooter extends SubsystemBase {

    /** Tunable number for the speed of the shooter wheels in m/s */
    private static final TunableNumber shootingSpeedMPS = new TunableNumber("Tuning/Shooter/TargetShooterSpeedMPS", ShooterConstants.DEFAULT_SPEED_MPS);
    /** Tunable number for the speed of the shooter wheels while the robot is intaking */
    private static final TunableNumber intakeSpeedMPS = new TunableNumber("Tuning/Shooter/TargetIntakeSpeed", -2.0);
    // Shooter motors PID constants
    private static final TunableNumber shooterPValue = new TunableNumber("Tuning/Shooter/ShooterPValue", ShooterConstants.PID_CONFIG.PROPORTIONAL);
    private static final TunableNumber shooterVValue = new TunableNumber("Tuning/Shooter/ShooterVValue", ShooterConstants.PID_CONFIG.VELOCITY);
    
    /** Tunable number for the position of the hood */
    private static final TunableNumber hoodPosition = new TunableNumber("Tuning/Shooter/TargetHoodPosition", ShooterConstants.HOOD_TARGET_POSITION);
    // Hood motor PID constants
    private static final TunableNumber hoodPValue = new TunableNumber("Tuning/Shooter/HoodPValue", ShooterConstants.HOOD_PID_CONFIG.PROPORTIONAL);
    private static final TunableNumber hoodDValue = new TunableNumber("Tuning/Shooter/HoodDValue", ShooterConstants.HOOD_PID_CONFIG.DERIVATIVE);

    /** Motor controlling the launching wheels */
    //one
    private final TalonFX shooterMotorOne;
    //two
    private final TalonFX shooterMotorTwo;
    //three
    private final TalonFX shooterMotorThree;

    /** Motor controlling the hood */
    private final TalonFX hoodMotor;

    private final DigitalInput hoodLimitSwitch;

    /** Request for controlling the motor in MPS */
    private final VelocityVoltage shooterVelocityVoltageRequest = new VelocityVoltage(0);
    private final PositionVoltage hoodPositionVoltageRequest = new PositionVoltage(ShooterConstants.HOOD_STARTING_POSITION)
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
                                        .withSlot(0);

    public Shooter() {
        shooterMotorOne = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_ONE_ID, Constants.RIO_CAN_BUS);
        shooterMotorTwo = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_TWO_ID, Constants.RIO_CAN_BUS);
        shooterMotorThree = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_THREE_ID, Constants.RIO_CAN_BUS);
        hoodMotor = new TalonFX(RioBusCANIds.HOOD_MOTOR_ID, Constants.RIO_CAN_BUS);
        hoodLimitSwitch = new DigitalInput(DigitalIDS.HOOD_LIMIT_SWITCH);

        updatePIDConfigs();

        // Set the hood's initial encoder position to 0
        // The hood should always start in the lowest possible possition
        hoodMotor.setPosition(ShooterConstants.HOOD_MIN_POSITION);

        Trigger limitSwitch = new Trigger(() -> hoodLimitSwitch.get());
        limitSwitch.onTrue(resetHoodEncoder());

        SmartDashboard.putBoolean(ShooterConstants.NT_APPLY_PID_BUTTON, false);
    }

    /** Used to configure motors and PID slots */
    private void updatePIDConfigs() {
        final TalonFXConfiguration shooterMotorCfg = new TalonFXConfiguration();
        shooterMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;
        shooterMotorCfg.Slot0.kS = ShooterConstants.PID_CONFIG.STATIC;
        shooterMotorCfg.Slot0.kV = shooterVValue.get();
        shooterMotorCfg.Slot0.kP = shooterPValue.get();
        shooterMotorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        shooterMotorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;
        
        final TalonFXConfiguration hoodMotorCfg = new TalonFXConfiguration();
        hoodMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodMotorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;
        hoodMotorCfg.Slot0.kP = hoodPValue.get();
        hoodMotorCfg.Slot0.kD = hoodDValue.get();

        // Applying the configuration
        shooterMotorOne.getConfigurator().apply(shooterMotorCfg);
        shooterMotorTwo.getConfigurator().apply(shooterMotorCfg);
        shooterMotorThree.getConfigurator().apply(shooterMotorCfg);
        hoodMotor.getConfigurator().apply(hoodMotorCfg);
    }

    /** Returns a command to shoot the fuel. */
    public Command start() {
        return Commands.run(() -> {
            // convert from target meters per second to wheel rotations per second
            double shootingwheelRPS = Util4828.metersPerSecondToWheelRPS(shootingSpeedMPS.get(), ShooterConstants.WHEEL_DIAMETER);
            shooterMotorOne.setControl(shooterVelocityVoltageRequest.withVelocity(-shootingwheelRPS));
            shooterMotorTwo.setControl(shooterVelocityVoltageRequest.withVelocity(shootingwheelRPS));
            shooterMotorThree.setControl(shooterVelocityVoltageRequest.withVelocity(shootingwheelRPS));
        }, this);
    }

    /** Returns a command that spins the shooter wheels during intake */
    public Command startIntake() {
        return Commands.run(() -> {
            // convert from target meters per second to wheel rotations per second
            double intakeWheelRPS = Util4828.metersPerSecondToWheelRPS(intakeSpeedMPS.get(), ShooterConstants.WHEEL_DIAMETER);
            shooterMotorOne.setControl(shooterVelocityVoltageRequest.withVelocity(-intakeWheelRPS));
            shooterMotorTwo.setControl(shooterVelocityVoltageRequest.withVelocity(intakeWheelRPS));
            shooterMotorThree.setControl(shooterVelocityVoltageRequest.withVelocity(intakeWheelRPS));
        }, this);
    }
    
    /** Returns a command to stop the shooting motors */
    public Command stop() {
        return Commands.runOnce(() -> {
            shooterMotorOne.stopMotor();
            shooterMotorTwo.stopMotor();
            shooterMotorThree.stopMotor();
        }, this);
    }

    /** Returns a command that raises the hood */
    public Command raiseHood() {
        return Commands.defer(
            () -> {
                return Commands.run(
                    () -> {
                        hoodMotor.setControl(hoodPositionVoltageRequest.withPosition(
                            MathUtil.clamp(hoodPosition.get(), ShooterConstants.HOOD_MAX_POSITION, ShooterConstants.HOOD_MIN_POSITION)));
                    }
                );
            },
            Collections.emptySet()
        );
    }

    /** Returns a command that lowers the hood to the minimum position */
    public Command lowerHood() {
        return Commands.run(() -> hoodMotor.setControl(hoodPositionVoltageRequest.withPosition(
            MathUtil.clamp(0.0, ShooterConstants.HOOD_MAX_POSITION, ShooterConstants.HOOD_MIN_POSITION))));
    }

    /** Resets the encoder of the hood motor */
    public Command resetHoodEncoder() {
        return Commands.runOnce(() -> hoodMotor.setPosition(ShooterConstants.HOOD_MIN_POSITION));
    }

    // private boolean resetEncoderRecently = false;

    @Override
    public void periodic() {
        // apply new PID configs 
        if (SmartDashboard.getBoolean(ShooterConstants.NT_APPLY_PID_BUTTON, false)) {
            updatePIDConfigs();
            SmartDashboard.putBoolean(ShooterConstants.NT_APPLY_PID_BUTTON, false); // reset btn
        }

        // if (!resetEncoderRecently && hoodLimitSwitch.get() == true){
        //     hoodMotor.setPosition(0);
        //     resetEncoderRecently = true;
        // } else {
        //     resetEncoderRecently = false;
        // }

        // output the current measured speed of the flywheel, for verification/tuning
        double actualMPS_ONE = shooterMotorOne.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber("Tuning/Shooter/ActualShooterSpeedMPSOne", actualMPS_ONE);
        double actualMPS_TWO = shooterMotorTwo.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber("Tuning/Shooter/ActualShooterSpeedMPSTwo", actualMPS_TWO);
        double actualMPS_THREE = shooterMotorThree.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber("Tuning/Shooter/ActualShooterSpeedMPSThree", actualMPS_THREE);

        // output actual hood position
        SmartDashboard.putNumber("Tuning/Shooter/ActualHoodPosition", hoodMotor.getPosition().getValueAsDouble());
    }

}