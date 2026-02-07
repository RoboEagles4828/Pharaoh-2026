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
    public enum State {
        SCORE,
        PASS_FROM_NEUTRAL,
        PASS_FROM_OPP,
        IDLE
    }

    private static final TunableNumber overrideTargetHood = new TunableNumber(ShooterConstants.NT_OVERRIDE_TARGET_HOOD, 0);
    private static final TunableNumber overrideTargetSpeed = new TunableNumber(ShooterConstants.NT_OVERRIDE_TARGET_SPEED, 0);

    private final CommandSwerveDrivetrain drivetrain;

    /** FLYWHEEL */
    private final TalonFX flywheelMotor;
    private final VelocityVoltage flywheelVelocityVoltageRequest = new VelocityVoltage(0);
    private boolean shouldFlywheelSpin = false;

    /** HOOD */
    // TODO - add TalonFX, control request, etc

    public State currentState;

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

        currentState = State.IDLE;
    }

    public void setState(State state) {
        currentState = state;
    }

    /** 
     * This command is the default command of this subsystem and runs at all times.
     * It updates the flywheel speed and hood angle according to the distance from the hub. 
     */
    public Command updateFlywheelAndHood() {
        return Commands.run(() -> {
            /** Update the flywheel speed */
            double targetSpeedMPS = 0.0;

            // If the flywheel is enabled...
            if (shouldFlywheelSpin) {
                // Use value from Elastic if override button in elastic is toggled
                if (SmartDashboard.getBoolean(ShooterConstants.NT_USE_DASHBOARD_VALUES, false)) {
                    targetSpeedMPS = overrideTargetSpeed.get();
                }   
                // Otherwise, calculate value normally based on distance from hub
                else {
                    switch(currentState) {
                        case IDLE:
                            targetSpeedMPS = getTargetFlywheelSpeedMPS(getDistanceToHub());
                            break;
                        case SCORE:
                            targetSpeedMPS = getTargetFlywheelSpeedMPS(getDistanceToHub());
                            break;
                        case PASS_FROM_NEUTRAL:
                            // TODO choose whether to aim top or bottom based on y position of pose
                            targetSpeedMPS = getTargetFlywheelSpeedMPS(getDistanceToTopPass());
                            break;
                        case PASS_FROM_OPP:
                            targetSpeedMPS = getTargetFlywheelSpeedMPS(getDistanceToTopPass());
                            break;
                    }
                }
            } 
            // If flywheel is not enabled, speed is 0.
            else {
                targetSpeedMPS = 0.0;
            }

            // convert the target meters per second to wheel rotations per second and set to motor
            double wheelRPS = Util4828.metersPerSecondToWheelRPS(targetSpeedMPS, ShooterConstants.WHEEL_DIAMETER);
            flywheelMotor.setControl(flywheelVelocityVoltageRequest.withVelocity(wheelRPS));
            SmartDashboard.putNumber(ShooterConstants.NT_TARGET_SPEED_MPS, targetSpeedMPS);


            /** Update the hood setting */
            double targetHood = 0.0;

            // Use value from Elastic if override button in elastic is toggled
            if (SmartDashboard.getBoolean(ShooterConstants.NT_USE_DASHBOARD_VALUES, false)) {
                targetHood = overrideTargetHood.get();
            }   
            // Otherwise, calculate value normally based on distance from hub
            else {
                switch(currentState) {
                    case IDLE:
                        targetHood = 0.0;
                        break;
                    case SCORE:
                        targetSpeedMPS = getHoodValue(getDistanceToHub());
                        break;
                    case PASS_FROM_NEUTRAL:
                        // TODO choose whether to aim top or bottom based on y position of pose
                        targetSpeedMPS = getHoodValue(getDistanceToTopPass()); // TODO - real value
                        break;
                    case PASS_FROM_OPP:
                        targetSpeedMPS = getHoodValue(getDistanceToTopPass()); // TODO - real value
                        break;
                }
            }
            
            //STEP2 - Apply to hood
            //TODO
            
            SmartDashboard.putNumber(ShooterConstants.NT_TARGET_HOOD, targetHood);
        }, this);
    }

    private double getDistanceToTopPass() {
        return 0.0;
    }

    private double getDistanceToBottomPass() {
        return 0.0;
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

    // Requests the flywheel to start spinning
    public void startSpinningFlywheel() {
        shouldFlywheelSpin = true;
    }

    // Requests the flywheel to stop spinning
    public void stopSpinningFlywheel() {
        shouldFlywheelSpin = false;
    }

    @Override
    public void periodic() {
        // output the current measured speed of the flywheel, for verification/tuning
        double actualSpeedMPS = flywheelMotor.getVelocity().getValueAsDouble() * Math.PI * ShooterConstants.WHEEL_DIAMETER;
        SmartDashboard.putNumber(ShooterConstants.NT_ACTUAL_SPEED_MPS, actualSpeedMPS);
    }
}