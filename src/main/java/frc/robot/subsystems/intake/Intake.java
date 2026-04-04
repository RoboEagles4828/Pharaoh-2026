package frc.robot.subsystems.intake;

import java.util.Collections;
import java.util.Set;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;

/** The intake subsystem controls the collection of fuel. */
public class Intake extends SubsystemBase {

    /** Motor that controls the deployment of the intake */
    private final TalonFX deployMotor;
    /** Motor that controls the ground pickup of the fuel */
    private final TalonFX intakeMotor;
    /** Motor that controls the transfer of fuel from the intake to the hopper */
    // private final TalonFX ninjaStarMotor;
    /** Limit switch that limits the retraction of the intake */
    // private final DigitalInput intakeLimitSwitch;

    // private final Trigger limitSwitch;

    /** Position control for the deployment of the intake */
    private final PositionVoltage deployPositionControl;
    private final MotionMagicVoltage deployMotionMagicControl;

    // PID constants for deploying the intake
    private static final TunableNumber deployPValue = new TunableNumber("Tuning/Intake/DeployPValue", IntakeConstants.DEPLOY_P);
    private static final TunableNumber deployDValue = new TunableNumber("Tuning/Intake/DeployDValue", IntakeConstants.DEPLOY_D);
    
    // PID constants for retracting the intake
    private static final TunableNumber retractPValue = new TunableNumber("Tuning/Intake/RetractPValue", IntakeConstants.RETRACT_P);
    private static final TunableNumber retractDValue = new TunableNumber("Tuning/Intake/RetractDValue", IntakeConstants.RETRACT_D);
    private static final TunableNumber retractGValue = new TunableNumber("Tuning/Intake/RetractGValue", IntakeConstants.RETRACT_G);

    // Positions to move the intake mechanism to
    private static final TunableNumber deployedPosition = new TunableNumber("Tuning/Intake/DeployPosition", IntakeConstants.DEPLOYED_POSITION);
    private static final TunableNumber raisedPosition = new TunableNumber("Tuning/Intake/RaisedPosition", IntakeConstants.RAISED_POSITION);
    
    private static final TunableNumber motionMagicVelocity = new TunableNumber("Tuning/Intake/MotionMagicVelocity",IntakeConstants.MOTION_MAGIC_VELOCITY);
    private static final TunableNumber motionMagicAcceleration = new TunableNumber("Tuning/Intake/MotionMagicAcceleration",IntakeConstants.MOTION_MAGIC_ACCELERATION);

    // Duty Cycle Speeds
    private static final TunableNumber intakeDutyCycle = new TunableNumber("Tuning/Intake/IntakeDutyCycle", IntakeConstants.INTAKE_DUTY_CYCLE); 
    // private static final TunableNumber ninjaStarDutyCycle = new TunableNumber("Tuning/Intake/NinjaStarDutyCycle", IntakeConstants.NINJA_STAR_DUTY_CYCLE);

    public Intake() {
        // Creating the motors on the rio can bus
        deployMotor = new TalonFX(RioBusCANIds.INTAKE_DEPLOY_MOTOR_ID, Constants.RIO_CAN_BUS);
        intakeMotor = new TalonFX(RioBusCANIds.INTAKE_MOTOR_ID, Constants.RIO_CAN_BUS);
        // ninjaStarMotor = new TalonFX(RioBusCANIds.NINJA_STAR_MOTOR_ID, Constants.RIO_CAN_BUS);
        // intakeLimitSwitch = new DigitalInput(DigitalIDS.INTAKE_LIMIT_SWITCH);

        // Configuring deploy motor (done separately because it has PID values that change)
        updatePID();

        // Configuring intake motor
        final TalonFXConfiguration intakeMotorCfg = new TalonFXConfiguration();
        intakeMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorCfg.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_RATIO;
        intakeMotorCfg.CurrentLimits = (new CurrentLimitsConfigs())
            .withSupplyCurrentLimit(40);

        // Configuring ninja star (hopper) motor
        // final TalonFXConfiguration ninjaStarCfg = new TalonFXConfiguration();
        //ninjaStarCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //ninjaStarCfg.Feedback.SensorToMechanismRatio = IntakeConstants.NINJA_STAR_GEAR_RATIO;
        //ninjaStarCfg.CurrentLimits = (new CurrentLimitsConfigs())
        //    .withSupplyCurrentLimit(40);

        // We start in the up position. Set the encoder so that 0.0 is the retracted position.
        deployMotor.setPosition(IntakeConstants.RAISED_POSITION);

        deployPositionControl = new PositionVoltage(0.0)
                                    .withEnableFOC(true)
                                    .withOverrideBrakeDurNeutral(true)
                                    .withSlot(0);
        deployMotionMagicControl = new MotionMagicVoltage(0.0)
                                    .withEnableFOC(true)
                                    .withOverrideBrakeDurNeutral(true)
                                    .withSlot(0);

        // limitSwitch = new Trigger(() -> !intakeLimitSwitch.get());
        //limitSwitch.onTrue(resetDeployEncoder());

        SmartDashboard.putBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false);
        SmartDashboard.putBoolean(IntakeConstants.NT_RESET_INTAKE_ENCODER_BUTTON, false);
    }

    public void updatePID() {
        // Configuring deploy motor
        final TalonFXConfiguration deployMotorCfg = new TalonFXConfiguration();
        deployMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotorCfg.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOY_GEAR_RATIO;
        // Use slot 0 for deployment PID values, slot 1 for retraction PID values
        deployMotorCfg.Slot0.kP = deployPValue.get();
        deployMotorCfg.Slot0.kD = deployDValue.get();
        deployMotorCfg.Slot1.kG = retractGValue.get();
        deployMotorCfg.Slot1.kP = retractPValue.get();
        deployMotorCfg.Slot1.kD = retractDValue.get();
        deployMotorCfg.CurrentLimits = (new CurrentLimitsConfigs())
            .withSupplyCurrentLimit(40);

        deployMotorCfg.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
        deployMotorCfg.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
            
        deployMotor.getConfigurator().apply(deployMotorCfg);

        // Reduce update frequency to reduce can% utilization...
        deployMotor.getPosition().setUpdateFrequency(50);
        deployMotor.getVelocity().setUpdateFrequency(50);
        deployMotor.getAcceleration().setUpdateFrequency(20);
        //ninjaStarMotor.getPosition().setUpdateFrequency(20);
        //ninjaStarMotor.getVelocity().setUpdateFrequency(20);
        //ninjaStarMotor.getAcceleration().setUpdateFrequency(20);
        intakeMotor.getPosition().setUpdateFrequency(20);
        intakeMotor.getVelocity().setUpdateFrequency(20);
        intakeMotor.getAcceleration().setUpdateFrequency(20);
    }

    /** Returns a command that deploys the intake */
    public Command deployIntake() {
        return Commands.defer(
            () -> {
                return Commands.run(() -> 
                    deployMotor.setControl(deployMotionMagicControl.withSlot(0).withPosition(deployedPosition.get()))
                );
            },
            Collections.emptySet()
        );
    }
    /** Returns a command that retracts the intake back into the robot */
    public Command retractIntake() {
        return Commands.defer(
            () -> {
                return Commands.run(() -> 
                    deployMotor.setControl(deployMotionMagicControl.withSlot(1).withPosition(raisedPosition.get()))
                );
            },
            Collections.emptySet()
        );
    }

    /** Returns a command that runs the intake motor */
    public Command startIntake() {
        return Commands.defer(
            () -> Commands.run(() -> {
            intakeMotor.set(intakeDutyCycle.get());
            }),
            Collections.emptySet());
    }    
    /** Returns a command that runs the ninja star motor */
//    public Command startNinjaStarMotor() {
  //      return Commands.defer(
    //        () -> Commands.run(() -> {
      //      ninjaStarMotor.set(ninjaStarDutyCycle.get());
        //    }),
          //  Collections.emptySet());
    //}

    public Command outtakeIntake() {
        return Commands.defer(
            () -> Commands.run(() -> {
            intakeMotor.set(-intakeDutyCycle.get());
            }),
            Collections.emptySet());
    }
 //   public Command outtakeNinja() {
   //     return Commands.defer(
     //       () -> Commands.run(() -> {
       //     ninjaStarMotor.set(-ninjaStarDutyCycle.get());
         //   }),
           // Collections.emptySet());
    //}

    /** Returns a command that stops the intake motor */
    public Command stopIntake() {
        return Commands.runOnce(() -> intakeMotor.stopMotor());
    }
    /** Returns a command that stops the ninja star motors */
 //   public Command stopNinjaStarMotor() {
   //     return Commands.runOnce(() -> ninjaStarMotor.stopMotor());
    //}

    /** Returns a command that performs the full retraction process: retracting, stopping intake and ninja stars */
    public Command stopAndRetract() {
        return Commands.defer(
            () -> Commands.parallel(
                stopIntake(),
                retractIntake()),
  //              stopNinjaStarMotor()),
            Set.of(this));
    }

    /** Returns a command that performs the full intake process: deploying, running intake and ninja stars */
    public Command intake() {
        return Commands.defer(
            () -> Commands.parallel(        
                deployIntake(),
                Commands.sequence(
                    Commands.waitSeconds(IntakeConstants.DELAY_BEFORE_START_SPINNING_SECONDS),
                    Commands.parallel(
                        startIntake())
                        //startNinjaStarMotor())
                )
            ),
            Set.of(this));
    }

    public Command outtake() {
        return Commands.defer(
            () -> Commands.parallel(        
                deployIntake(),
                Commands.sequence(
                    Commands.waitSeconds(IntakeConstants.DELAY_BEFORE_START_SPINNING_SECONDS),
                    Commands.parallel(
                        outtakeIntake())
                        //outtakeNinja())
                )
            ),
            Set.of(this));
    }

    /** Resets the encoder of the deploy motor */
    public Command resetDeployEncoder() {
        return Commands.runOnce(() -> deployMotor.setPosition(raisedPosition.get()));
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false)) {
            updatePID();
            SmartDashboard.putBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false);
        }

        if (SmartDashboard.getBoolean(IntakeConstants.NT_RESET_INTAKE_ENCODER_BUTTON, false)) {
            deployMotor.setPosition(raisedPosition.get()); // also reset encoder for testing
            SmartDashboard.putBoolean(IntakeConstants.NT_RESET_INTAKE_ENCODER_BUTTON, false);
        }

        SmartDashboard.putNumber("Tuning/Intake/DeployMotorPosition", deployMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putBoolean("Tuning/Intake/LimitSwitch", !intakeLimitSwitch.get());
        // SmartDashboard.putBoolean("Tuning/Intake/LimitTrigger", limitSwitch.getAsBoolean());
    }
}
