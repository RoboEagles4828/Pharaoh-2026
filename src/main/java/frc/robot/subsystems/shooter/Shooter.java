package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util4828;

/** The shooter subsystem controls the output of fuel. */
public class Shooter extends SubsystemBase {
    private static final TunableNumber overrideTargetHood = new TunableNumber(ShooterConstants.NT_OVERRIDE_TARGET_HOOD, 0);
    private static final TunableNumber overrideTargetSpeed = new TunableNumber(ShooterConstants.NT_OVERRIDE_TARGET_SPEED, 0);

    private final CommandSwerveDrivetrain drivetrain;

    /** FLYWHEEL */
    private final TalonFX flywheelMotor;
    private final VelocityVoltage flywheelVelocityVoltageRequest = new VelocityVoltage(0);
    private boolean isFlywheelSpinning = false;

    /** HOOD */
    // TODO

    public Shooter(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        flywheelMotor = new TalonFX(RioBusCANIds.SHOOTER_MOTOR_ID);

        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorCfg.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        motorCfg.Slot0.kV = ShooterConstants.PID_CONFIG.VELOCITY;
        motorCfg.Slot0.kP = ShooterConstants.PID_CONFIG.PROPORTIONAL;
        motorCfg.Slot0.kI = ShooterConstants.PID_CONFIG.INTEGRAL;
        motorCfg.Slot0.kD = ShooterConstants.PID_CONFIG.DERIVATIVE;
        flywheelMotor.getConfigurator().apply(motorCfg);
    }

    // Requests the flywheel to start spinning
    public void startSpinningFlywheel() {
        isFlywheelSpinning = true;
    }

    // Requests the flywheel to stop spinning
    public void stopSpinningFlywheel() {
        isFlywheelSpinning = false;
    }

    /** 
     * This command is the default command of this subsystem and runs at all times.
     * It updates the flywheel speed and hood angle 
     * according to the distance from the hub. 
     */
    public Command updateFlywheelAndHood() {
        return Commands.run(() -> {
            double targetSpeedMPS;
            double targetHood;

            if (SmartDashboard.getBoolean(ShooterConstants.NT_USE_DASHBOARD_VALUES, false)) {
                targetSpeedMPS = SmartDashboard.getNumber(ShooterConstants.NT_OVERRIDE_TARGET_SPEED, 0.0);
                targetHood = SmartDashboard.getNumber(ShooterConstants.NT_OVERRIDE_TARGET_HOOD, 0.0);
            }
            else {
                double distanceToHub = getDistanceToHub();
                targetSpeedMPS = isFlywheelSpinning ? getTargetFlywheelSpeedMPS(distanceToHub) : 0.0;
                targetHood = getHoodValue(distanceToHub);
            }

            // convert the target meters per second to wheel rotations per second and set
            double wheelRPS = Util4828.metersPerSecondToWheelRPS(targetSpeedMPS, ShooterConstants.WHEEL_DIAMETER);
            flywheelMotor.setControl(flywheelVelocityVoltageRequest.withVelocity(wheelRPS));

            // TODO - update hood, if we're attempting to shoot

            SmartDashboard.putNumber(ShooterConstants.NT_TARGET_HOOD, targetHood);
            SmartDashboard.putNumber(ShooterConstants.NT_TARGET_SPEED_MPS, targetSpeedMPS);
        }, this);
    }

    private double getDistanceToHub() {
        Pose2d drivePose = drivetrain.getState().Pose;
        Translation2d hubPose = Util4828.getHubLocation();
        return Util4828.getDistance(drivePose, hubPose);
    }

    private int getLookupIndexLower(double distance) {
        int index = (int)(2*distance);
        if (index < 0) {
            return 0;
        }
        if (index >= ShooterConstants.SHOOTER_LOOKUP_TABLE.length) {
            return ShooterConstants.SHOOTER_LOOKUP_TABLE.length - 1;
        }
        return index;
    }

    private int getLookupIndexUpper(double distance) {
        int index = (int)(2*distance) + 1;
        if (index < 0) {
            return 0;
        }
        if (index >= ShooterConstants.SHOOTER_LOOKUP_TABLE.length) {
            return ShooterConstants.SHOOTER_LOOKUP_TABLE.length - 1;
        }
        return index;
    }

    private double getTargetFlywheelSpeedMPS(double distance) {
        double x0 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexLower(distance)].getDistance();
        double x1 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexUpper(distance)].getDistance();
        double y0 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexLower(distance)].getFlywheelSpeed();
        double y1 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexUpper(distance)].getFlywheelSpeed();
        double m = (y1 - y0)/(x1 - x0);
        double targetSpeedMPS = y0 + m*(distance - x0);
        SmartDashboard.putNumber(ShooterConstants.NT_TARGET_SPEED_MPS, targetSpeedMPS);
        return targetSpeedMPS;
    }

    private double getHoodValue(double distance) {
        double x0 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexLower(distance)].getDistance();
        double x1 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexUpper(distance)].getDistance();
        double y0 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexLower(distance)].getHoodValue();
        double y1 = ShooterConstants.SHOOTER_LOOKUP_TABLE[getLookupIndexUpper(distance)].getHoodValue();
        double m = (y1 - y0)/(x1 - x0);
        double targetHoodValue01 = y0 + m*(distance - x0);
        SmartDashboard.putNumber(ShooterConstants.NT_TARGET_HOOD, targetHoodValue01);
        return targetHoodValue01;
    }

    @Override
    public void periodic() {
        // output the current measured speed of the flywheel, for verification/tuning
        double actualSpeedMPS = flywheelMotor.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualSpeedMPS);
    }
}