package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Climber extends SubsystemBase {
    private static TunableNumber climbUpDutyCycle = new TunableNumber(ClimberConstants.NT_CLIMB_UP_DUTY_CYCLE, ClimberConstants.DEFAULT_CLIMB_UP_DUTY_CYCLE);
    private static TunableNumber climbDownDutyCycle = new TunableNumber(ClimberConstants.NT_CLIMB_DOWN_DUTY_CYCLE, ClimberConstants.DEFAULT_CLIMB_DOWN_DUTY_CYCLE);

    public final TalonFX motor;

    public Climber() {
        motor = new TalonFX(Constants.RioBusCANIds.CLIMBER_MOTOR_ID);

        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Applying the configuration
        motor.getConfigurator().apply(motorCfg);
    }

    public Command climbUp() {
       return Commands.runOnce(() -> motor.set(climbUpDutyCycle.get()), this);
    }

    public Command climbDown() {
        return Commands.runOnce(() -> motor.set(climbDownDutyCycle.get()), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.stopMotor(), this);
    }
}
