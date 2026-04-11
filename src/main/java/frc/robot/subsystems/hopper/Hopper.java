package frc.robot.subsystems.hopper;

import java.util.Set;

import org.ejml.simple.ConvertToImaginaryException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.TunableNumber;

/** The hopper subsystem controls the feeding of fuel to the kicker and shooter subsystems. */
public class Hopper extends SubsystemBase {
    
    private TunableNumber hopperDutyCycle = new TunableNumber("Tuning/Hopper/HopperDutyCycle", HopperConstants.CONVEYOR_DUTY_CYCLE);

    /** Motor for the conveyer belts to each shooter */
    private final TalonFX conveyorMotor;

    public Hopper() {
        conveyorMotor = new TalonFX(Constants.RioBusCANIds.HOPPER_CONVEYOR_MOTOR_ID, Constants.RIO_CAN_BUS);
        final TalonFXConfiguration hopperMotorConfig = new TalonFXConfiguration();
        hopperMotorConfig.CurrentLimits = (new CurrentLimitsConfigs())
            .withSupplyCurrentLimit(HopperConstants.CURRENT_LIMIT);
        conveyorMotor.getConfigurator().apply(hopperMotorConfig);

        conveyorMotor.getPosition().setUpdateFrequency(4);
        conveyorMotor.getVelocity().setUpdateFrequency(4);
        conveyorMotor.getAcceleration().setUpdateFrequency(4);
    }

    /** Returns a command that runs the conveyer motors */
    public Command startConveyor() {
        return Commands.defer(
                () -> {
                    return Commands.run(() -> {
                        conveyorMotor.set(hopperDutyCycle.get());
                    });
                },
                Set.of(this));
    }
    /** Returns a command that runs the conveyor motor in the opposite direction */
    public Command startConveyorOuttake() {
        return Commands.defer(
                () -> {
                    return Commands.run(() -> {
                        conveyorMotor.set(-hopperDutyCycle.get());
                    });
                },
                Set.of(this));
    }
    /** Returns a command that stops the conveyer motor */
    public Command stopConveyor() {
        return Commands.defer(
                () -> {
                    return Commands.runOnce(() -> {
                        conveyorMotor.stopMotor();
                    });
                },
                Set.of(this));
    }
}
