package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Follow extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private double xSpeed;

  private final Debouncer debounceFilter = new Debouncer(0.25, DebounceType.kFalling);
  private final ShuffleboardTab followTab = Shuffleboard.getTab("Follow Tab");

  private final PIDController yawPIDController = new PIDController(0.25, 0.001, 0);
  private final PIDController xSpeedPIDController = new PIDController(0.1, 0, 0);

  public Follow(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    followTab.addNumber(
      "Traveled Distance: ", visionSubsystem::getDistance);
    followTab.addNumber("Target Yaw: ", visionSubsystem::getYaw);
    followTab.addNumber("xSpeed: ", this::getxSpeed);
  
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Running!");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopped: " + interrupted);
  }

  private double getxSpeed() {
    return xSpeed;
  }

  @Override
  public void execute() {
    if (!debounceFilter.calculate(visionSubsystem.hasValidTargets())) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double yaw = visionSubsystem.getYaw();
    yaw = (Math.abs(yaw) < 0.5) ? 0.0 : yaw;

    double targetYaw = driveSubsystem.getHeading() - yaw;
    double yawOutput = yawPIDController.calculate(driveSubsystem.getHeading(), targetYaw);
    double yawOutputRadians = Units.degreesToRadians(yawOutput);

    double distance = visionSubsystem.getDistance();
    xSpeed = xSpeedPIDController.calculate(distance, 1);

    driveSubsystem.drive(-xSpeed, 0, yawOutputRadians, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
