package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static final String IT_SPEED_RPS = "Tuning/Intake/SpeedRPS";

    private final TalonFX motor;

    public Intake() {
        motor = new TalonFX(Constants.RioBusCANIds.INTAKE_MOTOR_ID);

        /** Applying the configuration */
        SmartDashboard.putNumber(IT_SPEED_RPS, IntakeConstants.INTAKE_SPEED_DUTY_CYCLE);
    }

    public Command start() {
        return Commands.run(() -> motor.set(SmartDashboard.getNumber(IT_SPEED_RPS, IntakeConstants.INTAKE_SPEED_DUTY_CYCLE)), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }
}
