package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static final String NT_CLIMB_UP_DUTY_CYCLE = "Tuning/Climber/ClimbUpDutyCycle";
    private static final String NT_CLIMB_DOWN_DUTY_CYCLE = "Tuning/Climber/ClimbDownDutyCycle";

    public final TalonFX motor;

    public Climber() {
        motor = new TalonFX(Constants.RioBusCANIds.CLIMBER_MOTOR_ID);

        SmartDashboard.putNumber(NT_CLIMB_UP_DUTY_CYCLE, ClimberConstants.CLIMB_UP_DUTY_CYCLE);
        SmartDashboard.putNumber(NT_CLIMB_DOWN_DUTY_CYCLE, ClimberConstants.CLIMB_DOWN_DUTY_CYCLE);
    }

    public Command climbUp() {
       return Commands.runOnce(() -> motor.set(SmartDashboard.getNumber(NT_CLIMB_UP_DUTY_CYCLE, ClimberConstants.CLIMB_UP_DUTY_CYCLE)), this);
    }

    public Command climbDown() {
        return Commands.runOnce(() -> motor.set(SmartDashboard.getNumber(NT_CLIMB_DOWN_DUTY_CYCLE, ClimberConstants.CLIMB_DOWN_DUTY_CYCLE)), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }
}
