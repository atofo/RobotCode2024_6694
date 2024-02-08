// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeLauncherSubsystem;

public class IntakeLauncherCommands extends Command {

  private final IntakeLauncherSubsystem intakeLauncherSubsystem;
  private BooleanSupplier xButton;
  private BooleanSupplier aButton;
  private BooleanSupplier bButton;
  private BooleanSupplier RB;

  public IntakeLauncherCommands(IntakeLauncherSubsystem intakeLauncherSubsystem, BooleanSupplier xButton,
      BooleanSupplier aButton, BooleanSupplier bButton, BooleanSupplier RB) {
    this.intakeLauncherSubsystem = intakeLauncherSubsystem;
    this.xButton = xButton;
    this.aButton = aButton;
    this.bButton = bButton;
    this.RB = RB;
    addRequirements(intakeLauncherSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
