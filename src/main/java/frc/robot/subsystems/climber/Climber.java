package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public final TalonFX motor;

    public Climber() {
        motor = new TalonFX(Constants.RioBusCANIds.CLIMBER_MOTOR_ID);
    }

    public Command climbUp() {
       return Commands.runOnce(() -> motor.set(ClimberConstants.CLIMB_UP_DUTY_CYCLE));
    }

    public Command climbDown() {
        return Commands.runOnce(() -> motor.set(ClimberConstants.CLIMB_DOWN_DUTY_CYCLE));
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor());
    }
}
