package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SoundSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Follow extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private SoundSubsystem soundSubsystem;
  private double xSpeed;
  private boolean stopped = true;

  private final Debouncer stoppedFilter = new Debouncer(0.10);

  private final Debouncer debounceFilter = new Debouncer(0.25, DebounceType.kFalling);
  private final ShuffleboardTab followTab = Shuffleboard.getTab("Follow Tab");

  private final PIDController yawPIDController = new PIDController(0.25, 0.001, 0);
  private final PIDController xSpeedPIDController = new PIDController(0.1, 0, 0);

  public Follow(
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      SoundSubsystem soundSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.soundSubsystem = soundSubsystem;

    followTab.addNumber("Traveled Distance: ", visionSubsystem::getDistance);
    followTab.addNumber("Target Yaw: ", visionSubsystem::getYaw);
    followTab.addNumber("xSpeed: ", this::getxSpeed);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Running!");
    soundSubsystem.playSound1();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopped: " + interrupted);
    soundSubsystem.playSound2();
  }

  private double getxSpeed() {
    return xSpeed;
  }

  @Override
  public void execute() {
    if (!debounceFilter.calculate(visionSubsystem.hasValidTargets())) {
      driveSubsystem.drive(0, 0, 0);
      if (!stoppedFilter.calculate(stopped)) soundSubsystem.playSound2();
      stopped = true;
      return;
    }

    if (stoppedFilter.calculate(stopped)) soundSubsystem.playSound3();
    stopped = false;

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
