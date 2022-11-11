// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive;

  // The driver's controller
  private final XboxController driverController;

  // A chooser for autonomous commands
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    robotDrive = new DriveSubsystem();
    driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Configure the button bindings
    configureButtonBindings();
    configureTriggers();
    configureAutonomousChooser();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                -MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
            robotDrive));
  }

  private void configureAutonomousChooser() {
    // Add commands to the autonomous command chooser
   // chooser.addOption("Simple Path", AutonomousCommandHelper.getSimplAutonomousCommand(robotDrive));

    // Put the chooser on the dashboard
    SmartDashboard.putData(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driverController, Button.kA.value)
        .whenPressed(() -> robotDrive.zeroHeading());

    new JoystickButton(driverController, Button.kRightBumper.value)
        .whenPressed(() -> robotDrive.setTurboMode(true))
        .whenReleased(() -> robotDrive.setTurboMode(false));

    new JoystickButton(driverController, Button.kLeftBumper.value)
        .whenPressed(() -> robotDrive.setFieldRelative(false))
        .whenReleased(() -> robotDrive.setFieldRelative(true));

    new Trigger(() -> { return driverController.getRightTriggerAxis() > 0.75;})
        .whenActive(() -> robotDrive.setTurboMode(true))
        .whenInactive(() -> robotDrive.setTurboMode(false));
  }

  private void configureTriggers() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public void autonomousInit() {
  }

  public void teleopInit() {
  }

  public void disabledInit() {
  }

  public void periodic() {
  }

}
