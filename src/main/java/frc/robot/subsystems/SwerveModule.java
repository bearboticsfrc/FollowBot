// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule { 
  private final CANSparkMax driveMotor;
  private final WPI_VictorSPX turningMotor;

  private final RelativeEncoder driveEncoder;

  private final AnalogInput turningEncoder;

  private SwerveModuleState desiredState = new SwerveModuleState();
  
  private double zeroAngle;

 // private final PIDController drivePIDController =
 //     new PIDController(SwerveModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController =
      new ProfiledPIDController(
          SwerveModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              SwerveModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              SwerveModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

 // private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1,.5);
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param turningAnalogPort serial port for the analog input
   * @param driveEncoderReversed if the drive encoder should be reversed.
   * @param zeroAngle default offset of the encoder to make the wheel at the correct starting position.
   * @param moduleName Name (FL, FR, BL, BR)
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAnalogPort,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double zeroAngle,
      ShuffleboardLayout container,
      String moduleName) {
    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    if (driveMotor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk){
      SmartDashboard.putString("Idle Mode", "Error");
    }

    turningMotor = new WPI_VictorSPX(turningMotorChannel);
    turningMotor.setNeutralMode(NeutralMode.Coast);

    this.driveEncoder = driveMotor.getEncoder();

    this.turningEncoder = new AnalogInput(turningAnalogPort);
    
    this.zeroAngle = zeroAngle;

    driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec); // RPM to units per second
    driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRotationsPerMeter);

    driveMotor.setInverted(driveEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    if (SwerveModuleConstants.kSwerveModuleDebugMode) {
      container.addNumber(String.format("%s angle", moduleName), this::getAngle);
      container.addNumber(String.format("%s velocity", moduleName), driveEncoder::getVelocity);
      container.addNumber(String.format("%s drive current", moduleName), driveMotor::getOutputCurrent);
      container.addNumber(String.format("%s desired angle", moduleName), this::getDesiredAngleDegrees);
      container.addNumber(String.format("%s desired speed", moduleName), this::getDesiredSpeedMetersPerSecond);
      container.addNumber(String.format("%s drive value", moduleName), this::getCalculatedDriveValue);
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  public double getRawAngle() {
    return turningEncoder.getVoltage() * 360.0 / 5.0;
  }

  public double getAngle() {
    double temp =  getRawAngle() - zeroAngle;

    temp -= Math.floor(temp/360.0) * 360.0;

    return temp;
  }

  public double getDesiredSpeedMetersPerSecond() {
    return desiredState.speedMetersPerSecond;
  }

  public double getDesiredAngleDegrees() {
    return desiredState.angle.getDegrees();
  }

  public SwerveModuleState getOptimizedState() {
    return SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
  }

  public double getCalculatedDriveValue() {
    return getOptimizedState().speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    this.desiredState = desiredState;
    //if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
    //  stop();
    //  return;
    //}

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = getOptimizedState();

    // Calculate the drive output from the drive PID controller.
    //final double driveOutput =
    //    drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

   // final double driveFeedforward = driveFeedforward.calculate(state.speedMetersPerSecond);

    //double driveValue = MathUtil.clamp(driveOutput + state.speedMetersPerSecond, -0.2, 0.2);    

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        turningPIDController.calculate(Math.toRadians(getAngle()), state.angle.getRadians());
        
    driveMotor.set(getCalculatedDriveValue());
    turningMotor.set(turnOutput);
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
}

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    //driveEncoder.reset();
    //turningEncoder.reset();
    driveEncoder.setPosition(0);
  }

  public void outputToSmartDashboard() {
  }

}
