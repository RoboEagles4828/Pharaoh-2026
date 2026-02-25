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

public class Kicker extends SubsystemBase {
    private static final TunableNumber kickingSpeedMPS = new TunableNumber(KickerConstants.NT_KICKER_TARGET_SPEED_MPS, KickerConstants.DEFAULT_KICKER_SPEED_MPS);
    private static final TunableNumber intakeSpeedMPS = new TunableNumber("Tuning/Kicker/TargetIntakeSpeed", 2.0);
    private static final TunableNumber kickerPValue = new TunableNumber(KickerConstants.NT_KICKER_P_VALUE, KickerConstants.KICKER_PID_CONFIG.PROPORTIONAL);
    private static final TunableNumber kickerVValue = new TunableNumber(KickerConstants.NT_KICKER_V_VALUE, KickerConstants.KICKER_PID_CONFIG.VELOCITY);

    /** Motor controlling the kicker */
    private final TalonFX kickerMotor;
    private final VelocityVoltage kickerVelocityVoltageRequest = new VelocityVoltage(0);

    public Kicker() {
        kickerMotor = new TalonFX(RioBusCANIds.KICKER_MOTOR_ID, Constants.RIO_BUS_NAME);

        final TalonFXConfiguration kickerMotorCfg = new TalonFXConfiguration();
        kickerMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerMotorCfg.Feedback.SensorToMechanismRatio = KickerConstants.KICKER_GEAR_RATIO;
        kickerMotorCfg.Slot0.kV = kickerVValue.get();
        kickerMotorCfg.Slot0.kP = kickerPValue.get();

        kickerMotor.getConfigurator().apply(kickerMotorCfg);

        SmartDashboard.putBoolean("Tuning/Kicker/SpinDuringIntake", true);
    }

    public Command start() {
        return Commands.run(() -> {
            // convert from target meters per second to wheel rotations per second
            double kickerWheelRPS = Util4828.metersPerSecondToWheelRPS(kickingSpeedMPS.get(), KickerConstants.KICKER_WHEEL_DIAMETER);
            kickerMotor.setControl(kickerVelocityVoltageRequest.withVelocity(-kickerWheelRPS));
        }, this);
    }

    public Command startIntake() {
        return Commands.run(() -> {
            // toggleable on dashboard...
            if (SmartDashboard.getBoolean("Tuning/Kicker/SpinDuringIntake", false)) {
                // convert from target meters per second to wheel rotations per second
                double intakeWheelRPS = Util4828.metersPerSecondToWheelRPS(intakeSpeedMPS.get(), KickerConstants.KICKER_WHEEL_DIAMETER);
                kickerMotor.setControl(kickerVelocityVoltageRequest.withVelocity(-intakeWheelRPS));
            }
        }, this);
    }

    public Command stop() {
        return Commands.run(() -> {
            kickerMotor.stopMotor();
        }, this);
    }

    @Override
    public void periodic() {
        double actualKickerMPS = kickerMotor.getVelocity().getValueAsDouble() * Math.PI * KickerConstants.KICKER_WHEEL_DIAMETER;
        SmartDashboard.putNumber(KickerConstants.NT_ACTUAL_KICKER_SPEED_MPS, actualKickerMPS);
    }

}
