// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LeftClimber;

public class LeftEmergencyDown extends Command {
  private LeftClimber m_LeftClimberSubsystem;
  public LeftEmergencyDown(LeftClimber LeftClimberSubsystem) {
    this.m_LeftClimberSubsystem = LeftClimberSubsystem;
    addRequirements(LeftClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LeftClimberSubsystem.leftClimberEmergency();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LeftClimberSubsystem.leftClimberOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
