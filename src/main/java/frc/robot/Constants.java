// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    public enum TowerSide {
      LEFT, RIGHT
    }

    public static final AprilTagFields APRIL_TAG_FIELD_TYPE = AprilTagFields.k2026RebuiltAndymark;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(APRIL_TAG_FIELD_TYPE);
    public static final Field2d FIELD = new Field2d();
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6116, 4.0213); // AndyMark measurements - which NC uses
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9014, 4.0213); // AndyMark measurements - which NC uses
    
    public static final Translation2d BLUE_PASS_TOP = new Translation2d(2, 5.801);
    public static final Translation2d BLUE_PASS_BOTTOM = new Translation2d(2, 1.687);
    public static final Translation2d RED_PASS_TOP = new Translation2d(14.493, 5.801);
    public static final Translation2d RED_PASS_BOTTOM = new Translation2d(14.493, 1.687);
    public static final double FIELD_MIDPOINT_Y = 4.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class RioBusCANIds {
    // NOTE - 1 to 12 are reserved for the swerve drivetrain.
    public static int SHOOTER_MOTOR_ID = 13;
    public static int INTAKE_MOTOR_ID = 23;
    public static int KICKER_MOTOR_ID = 14;
  }

  public static String CANIVORE_NAME = "canivore";
  public static class CANivoreBusCANIds {
    public static int CLIMBER_MOTOR_ID = 22;
  }

  public static class RobotConstants {
    public static double DISTANCE_FRAME_EDGE_TO_CENTER_NO_BUMPERS_METERS = 0.3556;
    public static double ROBOT_BUMPER_SIZE_METERS = 0.0889;
    public static double DISTANCE_BUMPER_EDGE_TO_CENTER_METERS = DISTANCE_FRAME_EDGE_TO_CENTER_NO_BUMPERS_METERS + ROBOT_BUMPER_SIZE_METERS;
  }
}
