package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final TalonFX motor;

    public Intake() {
        motor = new TalonFX(0);
    }

    public Command start() {
        return Commands.runOnce(() -> motor.set(IntakeConstants.INTAKE_SPEED_DUTY_CYCLE), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }
}
