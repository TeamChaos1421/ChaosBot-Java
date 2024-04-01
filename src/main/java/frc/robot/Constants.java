// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
 
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCodriverControllerPort = 1;
  }

  public static class MotorConstants {
    public static final int kTopCANId = 53;
    public static final int kBotCANId = 22;
    public static final int kClimberCANId = 54;
    public static final int kShooterHighCANId = 21;
    public static final int kShooterMidCANId = 23;
    public static final int kShooterLowCANId = 20;
  }

  public static class PneumaticConstants {
    public static final int kPCMCANId = 1;
    public static final int kDumpForward = 0;
    public static final int kDumpReverse = 1;
  }

  public static class DrivetrainConstants {
    public static final double kTrackWidth = 0.6223;
    public static final double kTrackLength = 0.61595;
    public static final Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4.65);
    public static final Measure<Velocity<Angle>> kMaxRot = RadiansPerSecond.of(10);
  }

  // SWERVE MODULE CONSTANTS
  public static final class FrontLeftConstants {
    public static final String label = "Front Left";
    public static final int turnCANId = 59;
    public static final int driveCANId = 56;
    public static final double angOffset = 0;
    public static final int moduleNumber = 0;
    public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveCANId, turnCANId, angOffset, label, moduleNumber);
  }
  public static class FrontRightConstants {
    public static final String label = "Front Right";
    public static final int turnCANId = 61;
    public static final int driveCANId = 60;
    public static final double angOffset = Math.PI;
    public static final int moduleNumber = 1;
    public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveCANId, turnCANId, angOffset, label, moduleNumber);

  }
  public static class BackLeftConstants {
    public static final String label = "Back Left";
    public static final int turnCANId = 55;
    public static final int driveCANId = 52;
    public static final double angOffset = 0;
    public static final int moduleNumber = 2;
    public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveCANId, turnCANId, angOffset, label, moduleNumber);

  }
  public static class BackRightConstants {
    public static final String label = "Back Right";
    public static final int turnCANId = 57;
    public static final int driveCANId = 58;
    public static final double angOffset = 0;
    public static final int moduleNumber = 3;
    public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveCANId, turnCANId, angOffset, label, moduleNumber);

  }
}
