// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends MeasuredSubsystem {
  // Robot swerve modules
  private final SwerveModule frontLeft;
  private final SwerveModule backLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;

  // The gyro sensor
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(10);
  private double gyroOffetDegrees = 0.0;

  private double kMaxSpeed = 3; // 3 meters per second
  private double maxSpeed = kMaxSpeed;
  private boolean turboMode = false;
  private boolean fieldRelativeMode = true;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d());

  ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
  ShuffleboardTab swerveModulesTab = Shuffleboard.getTab("Swerve Modules");

  NetworkTableEntry maxSpeedEntry = tab.add("Drive Speed", kMaxSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(2, 1)
      .withProperties(Map.of("min", 0, "max", kMaxSpeed))
      .getEntry();

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    System.out.println("^^^^^^^^^^^^^^^^^^^^^ Gyro is " + (this.isGyroReady()? "Ready":"Not Ready"));
    
    frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningInputPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftZeroAngle,
        swerveModulesTab.getLayout("Front Left", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        "FL");

    backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftTurningInputPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftZeroAngle,
        swerveModulesTab.getLayout("Back Left", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        "BL");

    frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightTurningInputPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightZeroAngle,
        swerveModulesTab.getLayout("Front Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        "FR");

    backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightTurningInputPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightZeroAngle,
        swerveModulesTab.getLayout("Back Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        "BR");

    zeroHeading();
    //tab.addNumber("Gyro Angle", this::getHeading).withWidget(BuiltInWidgets.kGyro);
    tab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
    tab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    tab.addNumber("Pose rot", () -> odometry.getPoseMeters().getRotation().getDegrees());
  }

  public boolean isGyroReady() {
    return gyro.getState() == PigeonState.Ready;
  }
  
  public void setTurboMode(boolean mode) {
    if (mode) {
      turboMode = true;
      maxSpeed = Math.min(maxSpeed*2.0, kMaxSpeed);
    } else {
      turboMode = false;
      maxSpeed /= 2;
    }
    maxSpeedEntry.setDouble(maxSpeed);
  }

  public boolean getTurboMode() {
    return turboMode;
  }

  public void setFieldRelative(boolean mode) {
    fieldRelativeMode = mode;
  }

  public boolean getFieldRelative() {
    return fieldRelativeMode;
  }


  @Override
  public void monitored_periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d().plus(Rotation2d.fromDegrees(gyroOffetDegrees)),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
    outputToSmartDashboard();
    maxSpeed = maxSpeedEntry.getDouble(kMaxSpeed);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Vector2d getVelocity() {
    return new Vector2d(0.0, 0.0);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

 /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose, Rotation2d angle) {
    odometry.resetPosition(pose, angle);
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot, and sets
   * field relative to the current fieldRelativeMode setting.
   * 
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    this.drive(xSpeed, ySpeed, rot, fieldRelativeMode);
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot.
   * 
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative )  {
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
        * DriveConstants.kMaxSpeedMetersPerSecond;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().plus(Rotation2d.fromDegrees(gyroOffetDegrees)))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroOffetDegrees = 0.0;
    gyro.reset();
  }

 /** Zeroes the heading of the robot. */
 public void headingOffest(double offsetDegrees) {
   System.out.println("!!!!!!!!!!! Adding " + offsetDegrees);
   gyroOffetDegrees = offsetDegrees;
 // gyro.addFusedHeading(offsetDegrees);
}

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double temp = gyro.getRotation2d().getDegrees();
    temp -= Math.floor(temp / 360.0) * 360.0;
    return temp;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void outputToSmartDashboard() {
    frontLeft.outputToSmartDashboard();
    frontRight.outputToSmartDashboard();
    backLeft.outputToSmartDashboard();
    backRight.outputToSmartDashboard();
  }
}
