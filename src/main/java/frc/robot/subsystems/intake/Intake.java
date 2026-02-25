package frc.robot.subsystems.intake;

import java.util.Collections;
import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DigitalIDS;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;

public class Intake extends SubsystemBase {
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;
    private final TalonFX ninjaStarMotor;
    private final DigitalInput intakeLimitSwitch;

    private final PositionVoltage deployPositionControl;
    //private final VelocityVoltage intakeVelocityControl;
    //private final VelocityVoltage ninjaStarVelocityControl;

    private static final TunableNumber deployDownPValue = new TunableNumber(IntakeConstants.NT_DEPLOY_DOWN_P_VALUE, IntakeConstants.DEFAULT_DEPLOY_DOWN_P_VALUE);
    private static final TunableNumber deployDownDValue = new TunableNumber(IntakeConstants.NT_DEPLOY_DOWN_D_VALUE, IntakeConstants.DEFAULT_DEPLOY_DOWN_D_VALUE);
    private static final TunableNumber deployUpPValue = new TunableNumber(IntakeConstants.NT_DEPLOY_UP_P_VALUE, IntakeConstants.DEFAULT_DEPLOY_UP_P_VALUE);
    private static final TunableNumber deployUpGValue = new TunableNumber(IntakeConstants.NT_DEPLOY_UP_G_VALUE, IntakeConstants.DEFAULT_DEPLOY_UP_G_VALUE);
    private static final TunableNumber deployUpDValue = new TunableNumber(IntakeConstants.NT_DEPLOY_UP_D_VALUE, IntakeConstants.DEFAULT_DEPLOY_UP_D_VALUE);
    private static final TunableNumber deployedPosition = new TunableNumber(IntakeConstants.NT_DEPLOYED_POSITION, IntakeConstants.DEFAULT_DEPLOYED_POSITION);
    private static final TunableNumber raisedPosition = new TunableNumber(IntakeConstants.NT_RAISED_POSITION, IntakeConstants.DEFAULT_RAISED_POSITION); 
    // private static final TunableNumber intakeSValue = new TunableNumber(IntakeConstants.NT_INTAKE_S_VALUE, IntakeConstants.DEFAULT_INTAKE_S_VALUE);
    // private static final TunableNumber intakeVValue = new TunableNumber(IntakeConstants.NT_INTAKE_V_VALUE, IntakeConstants.DEFAULT_INTAKE_V_VALUE);
    // private static final TunableNumber intakePValue = new TunableNumber(IntakeConstants.NT_INTAKE_P_VALUE, IntakeConstants.DEFAULT_INTAKE_P_VALUE);
    // private static final TunableNumber intakeIValue = new TunableNumber(IntakeConstants.NT_INTAKE_I_VALUE, IntakeConstants.DEFAULT_INTAKE_I_VALUE);
    // private static final TunableNumber intakeDValue = new TunableNumber(IntakeConstants.NT_INTAKE_D_VALUE, IntakeConstants.DEFAULT_INTAKE_D_VALUE);
    // private static final TunableNumber intakeSpeedMPS = new TunableNumber(IntakeConstants.NT_INTAKE_SPEED_KEY, IntakeConstants.DEFAULT_INTAKE_SPEED_MPS); 

    public Intake() {
        deployMotor = new TalonFX(RioBusCANIds.INTAKE_DEPLOY_MOTOR_ID, Constants.RIO_BUS_NAME);
        intakeMotor = new TalonFX(RioBusCANIds.INTAKE_MOTOR_ID, Constants.RIO_BUS_NAME);
        ninjaStarMotor = new TalonFX(RioBusCANIds.NINJA_STAR_MOTOR_ID, Constants.RIO_BUS_NAME);
        intakeLimitSwitch = new DigitalInput(DigitalIDS.INTAKE_LIMIT_SWITCH);

        SmartDashboard.putBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false);
        SmartDashboard.putBoolean(IntakeConstants.NT_RESET_INTAKE_ENCODER_BUTTON, false);

        setMotorPID();

        // We start in the up position. Set the encoder so that 0.0 is the retracted position.
        deployMotor.setPosition(IntakeConstants.DEFAULT_RAISED_POSITION);

        deployPositionControl = new PositionVoltage(0.0)
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
                                        .withSlot(0);

        
        // intakeVelocityControl = new VelocityVoltage(0.0)
        //                                 .withEnableFOC(true)
        //                                 .withSlot(0);
        // ninjaStarVelocityControl = new VelocityVoltage(0.0)
        //                                 .withEnableFOC(true)
        //                                 .withSlot(0);
    }

    public void setMotorPID() {
        // Configuring deploy motor
        final TalonFXConfiguration deployMotorCfg = new TalonFXConfiguration();
        deployMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotorCfg.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOY_GEAR_RATIO;
        deployMotorCfg.Slot0.kP = deployDownPValue.get();
        deployMotorCfg.Slot0.kD = deployDownDValue.get();
        deployMotorCfg.Slot1.kG = deployUpGValue.get();
        deployMotorCfg.Slot1.kP = deployUpPValue.get();
        deployMotorCfg.Slot1.kD = deployUpDValue.get();
        deployMotorCfg.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        deployMotor.getConfigurator().apply(deployMotorCfg);

        // Configuring intake motor
        final TalonFXConfiguration intakeMotorCfg = new TalonFXConfiguration();
        intakeMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorCfg.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_RATIO;

        // Configuring ninja star (hopper) motor
        final TalonFXConfiguration ninjaStarCfg = new TalonFXConfiguration();
        ninjaStarCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ninjaStarCfg.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_NINJA_STAR_GEAR_RATIO;
    }

    public Command deployIntake() {
        return Commands.defer(
            () -> {
                return Commands.run(() -> 
                    deployMotor.setControl(deployPositionControl.withSlot(0).withPosition(deployedPosition.get()))
                );
            },
            Collections.emptySet()
        );
    }

    public Command retractIntake() {
        return Commands.defer(
            () -> {
                return Commands.run(() -> 
                    deployMotor.setControl(deployPositionControl.withSlot(1).withPosition(raisedPosition.get()))
                );
            },
            Collections.emptySet()
        );
    }

    //Start intake motor
    private static final TunableNumber intakeSpeed = new TunableNumber(IntakeConstants.NT_INTAKE_SPEED_KEY, IntakeConstants.DEFAULT_INTAKE_SPEED); 
    public Command startIntake() {
        return Commands.defer(
            () -> Commands.run(() -> {
            intakeMotor.set(intakeSpeed.get());
            }),
            Collections.emptySet());
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> intakeMotor.stopMotor());
    }
    
    //NinjaStartMotor
    private static final TunableNumber ninjaStarSpeed = new TunableNumber(IntakeConstants.NT_NINJA_STAR_SPEED_KEY, IntakeConstants.DEFAULT_NINJA_STAR_SPEED); 
    public Command startNinjaStarMotor() {
        return Commands.defer(
            () -> Commands.run(() -> {
            ninjaStarMotor.set(ninjaStarSpeed.get());
            }),
            Collections.emptySet());
    }

    public Command stopNinjaStarMotor() {
        return Commands.runOnce(() -> ninjaStarMotor.stopMotor());
    }

    public Command stopAndRetract() {
        return Commands.defer(
            () -> Commands.parallel(
                stopIntake(),
                retractIntake(),
                stopNinjaStarMotor()
        ), Set.of(this));
    }

    public Command intake() {
        return Commands.defer(
            () -> Commands.parallel(        
                deployIntake(),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.parallel(
                        startIntake(),
                        startNinjaStarMotor()
                    )
                )

        ), Set.of(this));
    }

    private boolean resetEncoderRecently = false;

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false)) {
            setMotorPID();
            SmartDashboard.putBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false);
        }

        if (SmartDashboard.getBoolean(IntakeConstants.NT_RESET_INTAKE_ENCODER_BUTTON, false)) {
            deployMotor.setPosition(raisedPosition.get()); // also reset encoder to 0.1 for testing
            SmartDashboard.putBoolean(IntakeConstants.NT_RESET_INTAKE_ENCODER_BUTTON, false);
        }

        // if (!resetEncoderRecently && intakeLimitSwitch.get() == true){
        //     deployMotor.setPosition(0);
        //     resetEncoderRecently = true;
        // } else {
        //     resetEncoderRecently = false;
        // }

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Tuning/Intake/DeployMotorPosition", deployMotor.getPosition().getValueAsDouble());
    }
}
