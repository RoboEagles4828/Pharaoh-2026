
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.RioBusCANIds;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX shooterMotorOne;
    private final TalonFX shooterMotorTwo;
    private final TalonFX shooterMotorThree;
    private final TalonFX hoodMotor;

    private final VelocityVoltage shooterVelocityRequest =
        new VelocityVoltage(0);

    private final PositionVoltage hoodPositionRequest =
        new PositionVoltage(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
            .withSlot(0);

    public ShooterIOTalonFX() {
        shooterMotorOne = new TalonFX(
            RioBusCANIds.SHOOTER_MOTOR_ONE_ID,
            Constants.RIO_CAN_BUS
        );

        shooterMotorTwo = new TalonFX(
            RioBusCANIds.SHOOTER_MOTOR_TWO_ID,
            Constants.RIO_CAN_BUS
        );

        shooterMotorThree = new TalonFX(
            RioBusCANIds.SHOOTER_MOTOR_THREE_ID,
            Constants.RIO_CAN_BUS
        );

        hoodMotor = new TalonFX(
            RioBusCANIds.HOOD_MOTOR_ID,
            Constants.RIO_CAN_BUS
        );

        hoodMotor.getPosition().setUpdateFrequency(50);
        hoodMotor.getVelocity().setUpdateFrequency(20);
        hoodMotor.getAcceleration().setUpdateFrequency(20);

        shooterMotorOne.getPosition().setUpdateFrequency(10);
        shooterMotorTwo.getPosition().setUpdateFrequency(10);
        shooterMotorThree.getPosition().setUpdateFrequency(10);
    }

    @Override
    public void configure(
        double shooterP,
        double shooterV,
        double hoodP,
        double hoodD
    ) {
        TalonFXConfiguration shooterCfg =
            new TalonFXConfiguration();

        shooterCfg.MotorOutput.NeutralMode =
            NeutralModeValue.Coast;

        shooterCfg.Feedback.SensorToMechanismRatio =
            ShooterConstants.SHOOTER_GEAR_RATIO;

        shooterCfg.Slot0.kS =
            ShooterConstants.PID_CONFIG.STATIC;

        shooterCfg.Slot0.kV = shooterV;
        shooterCfg.Slot0.kP = shooterP;
        shooterCfg.Slot0.kI =
            ShooterConstants.PID_CONFIG.INTEGRAL;

        shooterCfg.Slot0.kD =
            ShooterConstants.PID_CONFIG.DERIVATIVE;

        shooterCfg.CurrentLimits =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(
                    ShooterConstants.SHOOTER_CURRENT_LIMIT
                );

        TalonFXConfiguration hoodCfg =
            new TalonFXConfiguration();

        hoodCfg.MotorOutput.NeutralMode =
            NeutralModeValue.Brake;

        hoodCfg.Feedback.SensorToMechanismRatio =
            ShooterConstants.HOOD_GEAR_RATIO;

        hoodCfg.Slot0.kP = hoodP;
        hoodCfg.Slot0.kD = hoodD;

        hoodCfg.CurrentLimits =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(
                    ShooterConstants.HOOD_CURRENT_LIMIT
                );

        shooterMotorOne.getConfigurator().apply(shooterCfg);
        shooterMotorTwo.getConfigurator().apply(shooterCfg);
        shooterMotorThree.getConfigurator().apply(shooterCfg);
        hoodMotor.getConfigurator().apply(hoodCfg);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterOneVelocityRPS =
            shooterMotorOne.getVelocity().getValueAsDouble();

        inputs.shooterTwoVelocityRPS =
            shooterMotorTwo.getVelocity().getValueAsDouble();

        inputs.shooterThreeVelocityRPS =
            shooterMotorThree.getVelocity().getValueAsDouble();

        inputs.hoodPosition =
            hoodMotor.getPosition().getValueAsDouble();

        inputs.shooterOneCurrentAmps =
            shooterMotorOne.getStatorCurrent().getValueAsDouble();

        inputs.shooterTwoCurrentAmps =
            shooterMotorTwo.getStatorCurrent().getValueAsDouble();

        inputs.shooterThreeCurrentAmps =
            shooterMotorThree.getStatorCurrent().getValueAsDouble();

        inputs.hoodCurrentAmps =
            hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setShooterVelocity(
        double motorOneRPS,
        double motorTwoRPS,
        double motorThreeRPS
    ) {
        shooterMotorOne.setControl(
            shooterVelocityRequest.withVelocity(motorOneRPS)
        );

        shooterMotorTwo.setControl(
            shooterVelocityRequest.withVelocity(motorTwoRPS)
        );

        shooterMotorThree.setControl(
            shooterVelocityRequest.withVelocity(motorThreeRPS)
        );
    }

    @Override
    public void stopShooter() {
        shooterMotorOne.stopMotor();
        shooterMotorTwo.stopMotor();
        shooterMotorThree.stopMotor();
    }

    @Override
    public void setHoodTargetPosition(double position) {
        hoodMotor.setControl(
            hoodPositionRequest.withPosition(position)
        );
    }

    @Override
    public void setHoodEncoderPosition(double position) {
        hoodMotor.setPosition(position);
    }
}
