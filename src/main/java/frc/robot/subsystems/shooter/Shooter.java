package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;

/** The shooter subsystem controls the output of fuel. */
public class Shooter extends SubsystemBase {

    private static final String TOP_MOTOR_SPEED = "TopMotorSpeed";
    private static final String BOTTOM_MOTOR_SPEED = "BottomMotorSpeed";
    private final NetworkTable debugTable = NetworkTableInstance.getDefault().getTable("Debug");
    private final DoubleSubscriber topMotorSpeed = debugTable.getDoubleTopic(TOP_MOTOR_SPEED).subscribe(0.0);
    private final DoubleSubscriber bottomMotorSpeed = debugTable.getDoubleTopic(BOTTOM_MOTOR_SPEED).subscribe(0.0);

    /** Motor controlling the top line of wheels */
    private static TalonFX topMotor = new TalonFX(RioBusCANIds.SHOOTER_TOP_MOTOR_ID);
    /** Motor controlling the bottom line of wheels */
    private static TalonFX bottomMotor = new TalonFX(RioBusCANIds.SHOOTER_BOTTOM_MOTOR_ID);

    public Shooter() {
        /** Used to configure motors and PID slots. */
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorCfg.Slot0.kP = ShooterConstants.PID_CONFIG.PROPRTIONAL;
        motorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        motorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;

        debugTable.getDoubleTopic(TOP_MOTOR_SPEED).publish().setDefault(ShooterConstants.topShooterSpeed);
        debugTable.getDoubleTopic(BOTTOM_MOTOR_SPEED).publish().setDefault(ShooterConstants.bottomShooterSpeed);

        /** Applying the configuration to both motors. */
        topMotor.getConfigurator().apply(motorCfg);
        bottomMotor.getConfigurator().apply(motorCfg);
    }

    /**
     * Sets both motors to the speed provided.
     * 
     * @param speed
     */
    private void set(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }
    /**
     * Sets the top motor and bottom motor to the two speeds provided.
     * 
     * @param topMotorSpeed
     * @param bottomMotorSpeed
     */
    private void set(double topMotorSpeed, double bottomMotorSpeed) {
        topMotor.set(topMotorSpeed);
        bottomMotor.set(bottomMotorSpeed);
    }
    /**
     * Sets the speed of both motors to 0.
     */
    private void stopMotors() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    /** Command to shoot the fuel. */
    public Command shoot() {
        return Commands.run(
            () -> this.set(topMotorSpeed.get(), bottomMotorSpeed.get()));
    }
    /** Command to stop shooting fuel. */
    public Command stop() {
        return Commands.run(() -> this.stopMotors());
    }

}
