// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class Arm_manualSetpoint extends Command {
  private ArmSubsystem arm;

  public Arm_manualSetpoint(ArmSubsystem m_armSubsystem) {
    this.arm = m_armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Setpoint Adjusting");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.manualSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("Setpoint Adjusted");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
