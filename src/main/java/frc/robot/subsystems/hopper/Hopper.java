package frc.robot.subsystems.hopper;

import java.util.Collections;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Hopper extends SubsystemBase {
    
    private TunableNumber hopperDutyCycle = new TunableNumber(HopperConstants.NT_HOPPER_CONVEYOR_DUTY_CYCLE, HopperConstants.HOPPER_CONVEYOR_DEFAULT_DUTY_CYCLE);

    private final TalonFX conveyorMotor;

    public Hopper() {
        conveyorMotor = new TalonFX(Constants.RioBusCANIds.HOPPER_CONVEYOR_MOTOR_ID);
    }

    public Command startConveyor() {
        return Commands.defer(
                () -> {
                    return Commands.run(() -> {
                        conveyorMotor.set(hopperDutyCycle.get());
                    });
                },
                Collections.emptySet());
    }

    public Command stopConveyor() {
        return Commands.defer(
                () -> {
                    return Commands.run(() -> {
                        conveyorMotor.stopMotor();
                    });
                },
                Collections.emptySet());
    }
}
