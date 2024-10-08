// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Drivetrain Characterization
  public static final double ksVolts = 0.26313;
  public static final double kvVoltSecondsPerMeter = 0.046652;
  public static final double kaVoltSecondsSquaredPerMeter = 0.023248;
  public static final double kPDriveVel = 8.5;
  public static final double kTrackwidthMeters = 0.6588;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
        
  public static final double DRIVETRAIN_KS = 0.26313;
  public static final double DRIVETRAIN_KV = 0.046652;
  public static final double DRIVETRAIN_KA = 0.023248;
  public static final double DRIVETRAIN_KP = 9.0629E-06;
  public static final double DRIVETRAIN_KD = 0.001053;
  public static final double DRIVETRAIN_TRACK_WIDTH = 0.6588;

  // General Constants
  public static final int DRIVETRAIN_GEARBOX_RATIO = 0;
  public static final double SHOOTER_CONVERSION_FACTOR = (4096/600);

  public static final int GYRO_PORT = 0;
  public static final double GYRO_kVoltsPerDegreePerSecond = 0.0128;
  
  // Motor Controllers
  public static final int DRIVETRAIN_LEFT_FRONT = 40;
  public static final int DRIVETRAIN_LEFT_BACK = 30;
  public static final int DRIVETRAIN_RIGHT_FRONT = 20;
  public static final int DRIVETRAIN_RIGHT_BACK = 10;
  public static final int SHOOTER_RIGHT_TALON = 8;
  public static final int SHOOTER_LEFT_TALON = 7;
  public static final int SHOOTER_TURRET_TALON = 5;
  public static final int SHOOTER_FEED_TALON = 3;
  public static final int QUEUING_TALON = 6;
  public static final int INTAKE_ARM_TALON = 1;
  public static final int INTAKE_END_TALON= 4;

  // Solenoids
  public static final int SHOOTER_SOLENOID0 = 0;
  public static final int SHOOTER_SOLENOID3 = 1;
  public static final int INTAKE_SOLENOID1 = 2;
  public static final int INTAKE_SOLENOID2 = 3;

  // Joysticks
  public static final int DRIVER_CONTROLLER = 0;
  public static final int DRIVER_CONTROLLER_MOVE_AXIS = 1; // Change for your controller
  public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 4; // Change for your controller
}
