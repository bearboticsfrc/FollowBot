// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DT = 0.02; // 20ms framerate 50Hz
  public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz

  public static final class LEDConstants {
    public static final int kBlinkinPWMPort = 0;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 12;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 14;
    public static final int kBackRightDriveMotorPort = 15;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kBackRightTurningMotorPort = 9;

    public static final int kFrontLeftTurningInputPort = 3;
    public static final int kBackLeftTurningInputPort = 2;
    public static final int kFrontRightTurningInputPort = 0;
    public static final int kBackRightTurningInputPort = 1;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final double kFrontLeftZeroAngle = 48.0;
    public static final double kBackLeftZeroAngle = 272.0;
    public static final double kFrontRightZeroAngle = 134.0;
    public static final double kBackRightZeroAngle = 314.0;

    // Track Width Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.311;
    // Wheel Base Distance between front and back wheels on robot
    public static final double kWheelBase = 0.311;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 10;
  }

  public static final class SwerveModuleConstants {
    public static final boolean kSwerveModuleDebugMode = false;
    
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI * 4;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI * 16;

    // public static final int kEncoderCPR = 1024;
    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = 0.100; // Units.inchesToMeters(4);// 0.1016;
    public static final double kDriveGearReduction = 6.67 / 1.0;
    public static final double kDriveEncoderRotationsPerMeter = kWheelDiameterMeters * Math.PI / kDriveGearReduction;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * kDriveGearReduction / (double) kEncoderCPR;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRotationsPerMeter / 60;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = .1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 4;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

    public static final double kPXController = 4;
    public static final double kPYController = 4;
    public static final double kPThetaController = 8; // 32; // 32 worked at low speed

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // PID values for the rotation in the TargetDrive command
    public static final double kPTargetTurn = .02;
    public static final double kITargetTurn = 0.01;
    public static final double kDTargetTurn = 0.0;

    // P value used in the AutoRotate command
    public static final double kPAutoTurn = .01;
  }
}
