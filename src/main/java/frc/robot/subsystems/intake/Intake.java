package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Intake extends SubsystemBase {
    private static final TunableNumber intakeSpeedRPS = new TunableNumber(IntakeConstants.NT_INTAKE_SPEED_KEY, IntakeConstants.DEFAULT_INTAKE_SPEED);

    private final TalonFX motor;

    public Intake() {
        motor = new TalonFX(Constants.RioBusCANIds.INTAKE_MOTOR_ID);

        /** Used to configure motors and PID slots */
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Applying the configuration
        motor.getConfigurator().apply(motorCfg);
    }

    public Command start() {
        return Commands.run(() -> motor.set(intakeSpeedRPS.get()), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }
}
