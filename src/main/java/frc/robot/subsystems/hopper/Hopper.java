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
    private static final TunableNumber intakeDutyCycle = new TunableNumber(HopperConstants.NT_HOPPER_INTAKE_DUTY_CYCLE_KEY, HopperConstants.DEFAULT_INTAKE_SPEED_DUTY_CYCLE);
    private static final TunableNumber conveyorDutyCycle = new TunableNumber(HopperConstants.NT_HOPPER_CONVEYOR_DUTY_CYCLE_KEY, HopperConstants.DEFAULT_CONVEYOR_SPEED_DUTY_CYCLE);

    private final TalonFX intakeMotor;
    private final TalonFX conveyorMotor;

    public Hopper() {
        intakeMotor = new TalonFX(Constants.RioBusCANIds.HOPPER_INTAKE_MOTOR_ID);
        conveyorMotor = new TalonFX(Constants.RioBusCANIds.HOPPER_CONVEYOR_MOTOR_ID);

        final TalonFXConfiguration intakeCfg = new TalonFXConfiguration();
        intakeCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotor.getConfigurator().apply(intakeCfg);

        final TalonFXConfiguration conveyorCfg = new TalonFXConfiguration();
        conveyorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        conveyorMotor.getConfigurator().apply(conveyorCfg);
    }

    public Command startIntake() {
        return Commands.run(() -> intakeMotor.set(intakeDutyCycle.get()), this);
    }

    public Command startConveyor() {
        return Commands.run(() -> conveyorMotor.set(conveyorDutyCycle.get()), this);
    }

    public Command stop() {
        return Commands.runOnce(
            () -> {
                conveyorMotor.stopMotor();
                intakeMotor.stopMotor();
            },
            this
        );
    }
}
