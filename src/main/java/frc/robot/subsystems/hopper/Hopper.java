package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Hopper extends SubsystemBase {
    private static final TunableNumber speedDutyCycle = new TunableNumber(HopperConstants.NT_HOPPER_SPEED_KEY, HopperConstants.DEFAULT_HOPPER_SPEED_DUTY_CYCLE);

    private final TalonFX motor;

    public Hopper() {
        motor = new TalonFX(Constants.RioBusCANIds.HOPPER_MOTOR_ID);

        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Applying the configuration
        motor.getConfigurator().apply(motorCfg);
    }

    public Command start() {
        return Commands.run(() -> motor.set(speedDutyCycle.get()), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }
}
