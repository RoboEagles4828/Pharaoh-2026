package frc.robot.subsystems.intake;

import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util4828;

public class Intake extends SubsystemBase {
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;

    private final PositionVoltage deployPositionControl;
    private final VelocityVoltage intakeVelocityControl;

    private static final TunableNumber deployPValue = new TunableNumber(IntakeConstants.NT_DEPLOY_P_VALUE, IntakeConstants.DEFAULT_DEPLOY_P_VALUE);
    private static final TunableNumber deployIValue = new TunableNumber(IntakeConstants.NT_DEPLOY_I_VALUE, IntakeConstants.DEFAULT_DEPLOY_I_VALUE);
    private static final TunableNumber deployDValue = new TunableNumber(IntakeConstants.NT_DEPLOY_D_VALUE, IntakeConstants.DEFAULT_DEPLOY_D_VALUE);
    private static final TunableNumber deployPosition = new TunableNumber(IntakeConstants.NT_DEPLOY_POSITION, IntakeConstants.DEFAULT_DEPLOY_POSITION);
    private static final TunableNumber retractPosition = new TunableNumber(IntakeConstants.NT_RETRACT_POSITION, IntakeConstants.DEFAULT_RETRACT_POSITION);
    
    private static final TunableNumber intakeSValue = new TunableNumber(IntakeConstants.NT_INTAKE_S_VALUE, IntakeConstants.DEFAULT_INTAKE_S_VALUE);
    private static final TunableNumber intakeVValue = new TunableNumber(IntakeConstants.NT_INTAKE_V_VALUE, IntakeConstants.DEFAULT_INTAKE_V_VALUE);
    private static final TunableNumber intakePValue = new TunableNumber(IntakeConstants.NT_INTAKE_P_VALUE, IntakeConstants.DEFAULT_INTAKE_P_VALUE);
    private static final TunableNumber intakeIValue = new TunableNumber(IntakeConstants.NT_INTAKE_I_VALUE, IntakeConstants.DEFAULT_INTAKE_I_VALUE);
    private static final TunableNumber intakeDValue = new TunableNumber(IntakeConstants.NT_INTAKE_D_VALUE, IntakeConstants.DEFAULT_INTAKE_D_VALUE);
    private static final TunableNumber intakeSpeedMPS = new TunableNumber(IntakeConstants.NT_INTAKE_SPEED_KEY, IntakeConstants.DEFAULT_INTAKE_SPEED_MPS);

    public Intake() {
        deployMotor = new TalonFX(RioBusCANIds.INTAKE_DEPLOY_MOTOR_ID);
        intakeMotor = new TalonFX(RioBusCANIds.INTAKE_MOTOR_ID);

        SmartDashboard.putBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false);
        setMotorPID();

        deployPositionControl = new PositionVoltage(retractPosition.get())
                                        .withEnableFOC(true)
                                        .withOverrideBrakeDurNeutral(true)
                                        .withSlot(0);
        intakeVelocityControl = new VelocityVoltage(0.0)
                                        .withEnableFOC(true)
                                        .withSlot(0);
    }

    public void setMotorPID() {
        // Configuring deploy motor
        final TalonFXConfiguration deployMotorCfg = new TalonFXConfiguration();
        deployMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotorCfg.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOY_GEAR_RATIO;
        deployMotorCfg.Slot0.kP = deployPValue.get();
        deployMotorCfg.Slot0.kI = deployIValue.get();
        deployMotorCfg.Slot0.kD = deployDValue.get();
        deployMotor.getConfigurator().apply(deployMotorCfg);
        // Configuring intake motor
        final TalonFXConfiguration intakeMotorCfg = new TalonFXConfiguration();
        intakeMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorCfg.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_RATIO;
        intakeMotorCfg.Slot0.kS = intakeSValue.get();
        intakeMotorCfg.Slot0.kV = intakeVValue.get();
        intakeMotorCfg.Slot0.kP = intakePValue.get();
        intakeMotorCfg.Slot0.kI = intakeIValue.get();
        intakeMotorCfg.Slot0.kD = intakeDValue.get();
        intakeMotor.getConfigurator().apply(intakeMotorCfg);
    }

    public Command deployIntake() {
        return Commands.runOnce(() -> deployMotor.setControl(deployPositionControl.withPosition(deployPosition.get())), this);
    }
    public Command retractIntake() {
        return Commands.runOnce(() -> deployMotor.setControl(deployPositionControl.withPosition(retractPosition.get())), this);
    }

    public Command startIntake() {
        double wheelRPS = Util4828.metersPerSecondToWheelRPS(intakeSpeedMPS.get(), IntakeConstants.WHEEL_DIAMETER);
        return Commands.run(() -> intakeMotor.setControl(intakeVelocityControl.withVelocity(wheelRPS)), this);
    }
    public Command stopIntake() {
        return Commands.runOnce(() -> intakeMotor.stopMotor(), this);
    }

    public Command stopAndRetract() {
        return Commands.sequence(
            stopIntake(),
            retractIntake()
        );
    }

    public Command intake() {
        return Commands.defer(
            () -> Commands.sequence(
                //deployIntake(),
                startIntake()
        ), Set.of(this));
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false)) {
            setMotorPID();
            SmartDashboard.putBoolean(IntakeConstants.NT_UPDATE_INTAKE_PID_BUTTON, false);
        }

        double wheelRPS = Util4828.metersPerSecondToWheelRPS(intakeSpeedMPS.get(), IntakeConstants.WHEEL_DIAMETER);
        SmartDashboard.putNumber("Target Speed", wheelRPS);

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake/DeployMotorPosition", deployMotor.getPosition().getValueAsDouble());
        
        double actualMPS = intakeMotor.getVelocity().getValueAsDouble() * Math.PI * IntakeConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber("Actual Intake Speed MPS", actualMPS);
    }
}
