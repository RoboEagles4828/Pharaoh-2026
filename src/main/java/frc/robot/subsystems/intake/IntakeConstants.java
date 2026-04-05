package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final double DEPLOY_GEAR_RATIO = 20.0; 
    public static final double INTAKE_GEAR_RATIO = 3.0;
    
    public static final double DELAY_BEFORE_START_SPINNING_SECONDS = 0.33;
    public static final double AGITATION_DELAY_SECONDS = 0.25;
    
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(2.25);
    public static final double INTAKE_DUTY_CYCLE = 0.8;

    public static final double INTAKE_CURRENT_LIMIT = 40.0;
    public static final double DEPLOY_CURRENT_LIMIT = 40.0;
    public static final double DEPLOY_P = 20.0;
    public static final double DEPLOY_D = 0.1;

    public static final double RETRACT_P = 64.0;
    public static final double RETRACT_D = 0.2;
    public static final double RETRACT_G = 0.0;

    public static final double MOTION_MAGIC_VELOCITY = 3.0; // in mechanism rotations per second
    public static final double MOTION_MAGIC_ACCELERATION = 15.0; // in mechanism rotations per second squared

    public static final double DEPLOYED_POSITION = 0.0; // in mechanism rotations
    public static final double RAISED_POSITION = 0.333251953125; // in mechanism rotations

    public static final String NT_UPDATE_INTAKE_PID_BUTTON = "Tuning/Intake/UpdatePIDButton";
    public static final String NT_RESET_INTAKE_ENCODER_BUTTON = "Tuning/Intake/ResetDeployEncoder";
}
