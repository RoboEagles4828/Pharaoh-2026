package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Climber extends SubsystemBase {
    /** Motor that controls the climber */
    private final TalonFX motor;
    /** PID control to specific positions; default position is the starting position */
    private final PositionVoltage positionControl;

    /** Tunable number for the duty cycle for climbing up */
    private static TunableNumber climbUpDutyCycle = new TunableNumber(ClimberConstants.NT_CLIMB_UP_DUTY_CYCLE, ClimberConstants.DEFAULT_CLIMB_UP_DUTY_CYCLE);
    /** Tunable number for the duty cycle for climbing down */
    private static TunableNumber climbDownDutyCycle = new TunableNumber(ClimberConstants.NT_CLIMB_DOWN_DUTY_CYCLE, ClimberConstants.DEFAULT_CLIMB_DOWN_DUTY_CYCLE);

    /** Tunable number for the p-value of the climber motor */
    private static TunableNumber pValue = new TunableNumber(ClimberConstants.NT_CLIMBER_P_VALUE, ClimberConstants.DEFAULT_CLIMBER_P_VALUE);
    /** Tunable number for the i-value of the climber motor */
    private static TunableNumber iValue = new TunableNumber(ClimberConstants.NT_CLIMBER_I_VALUE, ClimberConstants.DEFAULT_CLIMBER_I_VALUE);
    /** Tunable number for the d-value of the climber motor */
    private static TunableNumber dValue = new TunableNumber(ClimberConstants.NT_CLIMBER_D_VALUE, ClimberConstants.DEFAULT_CLIMBER_D_VALUE);

    /** Tunable number for the starting position of the climber */
    private TunableNumber climbStartingPosition = new TunableNumber(ClimberConstants.NT_CLIMB_STARTING_POSITION, ClimberConstants.DEFAULT_CLIMB_STARTING_POSITION);
    /** Tunable number for the peak (tallest) position of the climber */
    private TunableNumber climbPeakPosition = new TunableNumber(ClimberConstants.NT_CLIMB_PEAK_POSITION, ClimberConstants.DEFAULT_CLIMB_PEAK_POSITION);
    /** Tunable number for the final position of the climber where it will hang*/
    // TODO might be unnecessary if the robot will pull itself up all the way but I thought that was unnecessary and this is safer
    private TunableNumber climbFinalPosition = new TunableNumber(ClimberConstants.NT_CLIMB_FINAL_POSITION, ClimberConstants.DEFAULT_CLIMB_FINAL_POSITION);

    /** Constructs a climber subsystem */
    public Climber() {
        motor = new TalonFX(Constants.CANivoreBusCANIds.CLIMBER_MOTOR_ID, Constants.CANIVORE_NAME);
        
        // Configuring the motor
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorCfg.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;
        motorCfg.Slot0.kP = pValue.get();
        motorCfg.Slot0.kI = iValue.get();
        motorCfg.Slot0.kD = dValue.get();
        // Applying the configuration
        motor.getConfigurator().apply(motorCfg);


        // Setting the default position control to the starting position
        positionControl = new PositionVoltage(climbStartingPosition.get())
                                .withEnableFOC(true)
                                .withOverrideBrakeDurNeutral(true)
                                .withSlot(0);
    }

    /** Sets the motor speed */
    public void setMotorSpeed(double speed){
        motor.set(speed);
    }
    /** Command to move the climber up at the constant duty cycle */
    public Command climbUp() {
       return this.run(() -> motor.set(climbUpDutyCycle.get()));
    }
    /** Command to move the climber down at the constant duty cycle */
    public Command climbDown() {
        return this.run(() -> motor.set(climbDownDutyCycle.get()));
    }
    /** Command to stop the climber motor */
    public Command stop() {
        return this.runOnce(() -> motor.stopMotor());
    }

    /** Gets the current position of the climber */
    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }

    /** Climbs to the peak position */
    public Command climbToPeak() {
        return this.run(
            () -> motor.setControl(positionControl.withPosition(climbPeakPosition.get()))
        );
    }
    /** Climbs to the final position where the robot will hang */
    public Command climbToFinal() {
        return this.run(
            () -> motor.setControl(positionControl.withPosition(climbFinalPosition.get()))
        );
    }
    /** Retracts the climber back to the starting position */
    public Command retractClimb() {
        return this.run(
            () -> motor.setControl(positionControl.withPosition(climbStartingPosition.get()))
        );
    }

    /** Full climbing sequence */
    public Command climb(){
        return Commands.sequence(
            climbToPeak(),
            climbToFinal()
            // climbToPeak(),
            // retractClimb()
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber Position", getPosition());
    }
    
}
