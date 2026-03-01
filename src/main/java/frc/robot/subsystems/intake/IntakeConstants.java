package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static double DEPLOY_GEAR_RATIO = 20.0; 
    public static double INTAKE_GEAR_RATIO = 4.0; 
    public static double NINJA_STAR_GEAR_RATIO = 5.0;

    public static double WHEEL_DIAMETER = Units.inchesToMeters(2.25);

    public static double INTAKE_DUTY_CYCLE = 0.9;
    public static double NINJA_STAR_DUTY_CYCLE = -0.9;

    public static double DEPLOY_P = 8.0;
    public static double DEPLOY_D = 0.1;

    public static double RETRACT_P = 15.0;
    public static double RETRACT_D = 0.2;
    public static double RETRACT_G = 2.0;

    public static double DEPLOYED_POSITION = 0.0; // in mechanism rotations
    public static double RAISED_POSITION = 0.21; // in mechanism rotations

    public static final String NT_UPDATE_INTAKE_PID_BUTTON = "Tuning/Intake/UpdatePIDButton";
    public static final String NT_RESET_INTAKE_ENCODER_BUTTON = "Tuning/Intake/ResetDeployEncoder";
}
