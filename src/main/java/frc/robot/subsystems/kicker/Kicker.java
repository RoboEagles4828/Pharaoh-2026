package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util4828;

/** The kicker subsystem controls the  kicker wheels that feed the shooter. */
public class Kicker extends SubsystemBase {
    private static final TunableNumber kickingSpeedMPS = new TunableNumber("Tuning/Kicker/TargetKickerSpeedMPS", KickerConstants.DEFAULT_KICKER_SPEED_MPS);
    private static final TunableNumber intakeSpeedMPS = new TunableNumber("Tuning/Kicker/TargetKickerIntakingSpeed", 0.0);
    private static final TunableNumber kickerPValue = new TunableNumber("Tuning/Kicker/KickerPValue", KickerConstants.kP);
    private static final TunableNumber kickerVValue = new TunableNumber("Tuning/Kicker/KickerVValue", KickerConstants.kV);

    /** Motor controlling the kicker */
    private final TalonFX kickerMotor;

    /** Velocity voltage request controlling the speed of the kicker motor */
    private final VelocityVoltage kickerVelocityVoltageRequest = new VelocityVoltage(0);

    public Kicker() {
        // Creating the kicker motor on the rio can bus
        kickerMotor = new TalonFX(RioBusCANIds.KICKER_MOTOR_ID, Constants.RIO_CAN_BUS);

        // Configuring the kicker motor
        final TalonFXConfiguration kickerMotorCfg = new TalonFXConfiguration();
        kickerMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerMotorCfg.Feedback.SensorToMechanismRatio = KickerConstants.KICKER_GEAR_RATIO;
        kickerMotorCfg.Slot0.kV = kickerVValue.get();
        kickerMotorCfg.Slot0.kP = kickerPValue.get();
        // Applying the configuration
        kickerMotor.getConfigurator().apply(kickerMotorCfg);

        SmartDashboard.putBoolean("Tuning/Kicker/SpinDuringIntake", true);
    }

    /** Returns a command that runs the kicker wheels */
    public Command start() {
        return Commands.run(() -> {
            // convert from target meters per second to wheel rotations per second
            double kickerWheelRPS = Util4828.metersPerSecondToWheelRPS(kickingSpeedMPS.get(), KickerConstants.KICKER_WHEEL_DIAMETER);
            kickerMotor.setControl(kickerVelocityVoltageRequest.withVelocity(-kickerWheelRPS));
        }, this);
    }

    /** Returns a command that runs the kciker wheels while intaking; toggleable on dashboard */
    // TODO test whether this is worth doing
    public Command startIntake() {
        return Commands.run(() -> {
            if (SmartDashboard.getBoolean("Tuning/Kicker/SpinDuringIntake", false)) {
                // convert from target meters per second to wheel rotations per second
                double kickerWheelRPS = Util4828.metersPerSecondToWheelRPS(intakeSpeedMPS.get(), KickerConstants.KICKER_WHEEL_DIAMETER);
                kickerMotor.setControl(kickerVelocityVoltageRequest.withVelocity(-kickerWheelRPS));
            }
        }, this);
    }

    /** Returns a command that stops the kicker motors */
    public Command stop() {
        return Commands.run(() -> {
            kickerMotor.stopMotor();
        }, this);
    }

    @Override
    public void periodic() {
        double actualKickerMPS = kickerMotor.getVelocity().getValueAsDouble() * Math.PI * KickerConstants.KICKER_WHEEL_DIAMETER;
        SmartDashboard.putNumber("Tuning/Kicker/ActualKickerSpeedMPS", actualKickerMPS);
    }
}
