package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

/** The climber subsystem controls the climbing of the robot during autonomous and endgame. */
public class Climber extends SubsystemBase {
    /** Motor that controls the climber */
    private final TalonFX climbMotor;
    /** PID control to specific positions; default position is the starting position */
    private final PositionVoltage positionControl;
    /** Hall effect sensor limitting the retraction of the climber */
    // private final DigitalInput climberHallEffect;
    /** Trigger that is activated when the limit switch is pressed */
    // private final Trigger climberLimitSensor;

    // Tunable numbers for duty cycle speeds
    private static TunableNumber climbUpDutyCycle = new TunableNumber("Tuning/Climber/ClimbUpDutyCycle", -ClimberConstants.DEFAULT_DUTY_CYCLE);
    private static TunableNumber climbDownDutyCycle = new TunableNumber("Tuning/Climber/ClimbDownDutyCycle", ClimberConstants.DEFAULT_DUTY_CYCLE);

    // Tunable numbers for PID values
    private static TunableNumber pValue = new TunableNumber("Tuning/Climber/ClimberP", ClimberConstants.kP);
    private static TunableNumber iValue = new TunableNumber("Tuning/Climber/ClimberI", ClimberConstants.kI);
    private static TunableNumber dValue = new TunableNumber("Tuning/Climber/ClimberD", ClimberConstants.kD);
    
    /** Tunable number for the peak (tallest) position of the climber */
    private TunableNumber climbPosition = new TunableNumber("Tuning/Climber/TallestPosition", ClimberConstants.CLIMB_POSITION);
    
    /** Constructs a climber subsystem */
    public Climber() {
        climbMotor = new TalonFX(Constants.RioBusCANIds.CLIMBER_MOTOR_ID, Constants.RIO_CAN_BUS);
        
        updatePIDConfigs();

        // assume we start in the lowered position; seed encoder to 0!
        climbMotor.setPosition(ClimberConstants.START_POSITION);

        // Setting the default position control to the starting position
        positionControl = new PositionVoltage(ClimberConstants.START_POSITION)
                                .withEnableFOC(true)
                                .withOverrideBrakeDurNeutral(true)
                                .withSlot(0);
        
        // climberHallEffect = new DigitalInput(Constants.DigitalIDS.CLIMBER_HALL_EFFECT);
        // climberLimitSensor = new Trigger(() -> !climberHallEffect.get());
        // climberLimitSensor.onTrue(resetClimberEncoder());

        if (Constants.debugMode)
        // Putting buttons on smart dashboard to zero the encoder and apply PID changes
            SmartDashboard.putBoolean("Tuning/Climber/ApplyPIDButton", false);
        SmartDashboard.putBoolean("Tuning/Climber/ZeroEncoderBtn", false);
    }

    /** Updates PID constants of the climber motor */
    private void updatePIDConfigs() {
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorCfg.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;
        motorCfg.Slot0.kP = pValue.get();
        motorCfg.Slot0.kI = iValue.get();
        motorCfg.Slot0.kD = dValue.get();
        motorCfg.CurrentLimits = (new CurrentLimitsConfigs())
            .withSupplyCurrentLimit(ClimberConstants.CURRENT_LIMIT);
        climbMotor.getConfigurator().apply(motorCfg);
        
        climbMotor.getPosition().setUpdateFrequency(30);
        climbMotor.getVelocity().setUpdateFrequency(10);
        climbMotor.getAcceleration().setUpdateFrequency(10);
    }

    /** Command to stop the climber motor */
    public Command stop() {
        return this.runOnce(() -> climbMotor.stopMotor());
    }

    // Moving via duty cycle
    /** Command to move the climber up at the constant duty cycle */
    public Command climbUpDutyCycle() {
       return this.run(() -> climbMotor.set(climbUpDutyCycle.get()));
    }
    /** Command to move the climber down at the constant duty cycle */
    public Command climbDownDutyCycle() {
        return this.run(() -> climbMotor.set(climbDownDutyCycle.get()));
    }

    // Moving via position control
    /** Climbs to the peak position */
    public Command extend() {
        return this.runOnce(() -> {
            climbMotor.setControl(positionControl.withPosition(climbPosition.get()));
        });
    }
    /** Retracts the climber back to the starting position */
        public Command retract() {
        return this.runOnce(
            () -> climbMotor.setControl(positionControl.withPosition(ClimberConstants.START_POSITION))
        );
    }

    /** Gets the current position of the climber */
    public double getPosition(){
        return climbMotor.getPosition().getValueAsDouble();
    }

    /** Resets the climber encoder position to the starting */
    public Command resetClimberEncoder() {
        return new InstantCommand(() -> climbMotor.setPosition(ClimberConstants.START_POSITION));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition());

        if (Constants.debugMode) {
            // Putting a button on the Dashboard to apply PID changes when pressed, and then resetting the button
            if (SmartDashboard.getBoolean("Tuning/Climber/ApplyPIDButton", false)) {
                updatePIDConfigs();
                SmartDashboard.putBoolean("Tuning/Climber/ApplyPIDButton", false);
            }
            // Putting a button on the Dashboard to zero the climber encoder when pressed, and then resetting the button
            if (SmartDashboard.getBoolean("Tuning/Climber/ZeroEncoderBtn", false)) {
                climbMotor.setPosition(0.0);
                SmartDashboard.putBoolean("Tuning/Climber/ZeroEncoderBtn", false);
            }
        }

        // SmartDashboard.putBoolean("Tuning/Climber/HallEffect", !climberHallEffect.get());
        // SmartDashboard.putBoolean("Tuning/Climber/LimitSensor", climberLimitSensor.getAsBoolean());
    }
}


