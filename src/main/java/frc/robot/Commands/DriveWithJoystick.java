// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class DriveWithJoystick extends Command {
  /** Creates a new DriveWithJoystick. */

  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier joystickX;
  private final DoubleSupplier joystickY;
  private final DoubleSupplier joystickZ;

  public DriveWithJoystick(DrivetrainSubsystem drivetrain, DoubleSupplier joystickX, DoubleSupplier joystickY,
      DoubleSupplier joystickZ) {
    this.drivetrain = drivetrain;
    this.joystickX = joystickX;
    this.joystickY = joystickX;
    this.joystickZ = joystickX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Boolean Inverted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.drive(joystickX, joystickY, joystickZ);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
