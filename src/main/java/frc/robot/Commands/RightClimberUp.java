// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RightClimber;

public class RightClimberUp extends Command {
  private RightClimber m_RightClimberSubsystem;
  public RightClimberUp(RightClimber RightClimberSubsystem) {
    this.m_RightClimberSubsystem = RightClimberSubsystem;
    addRequirements(RightClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RightClimberSubsystem.rightClimberUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_RightClimberSubsystem.rightClimberOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}