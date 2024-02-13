// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.testLauncherPID_1;

public class LauncherPIDCommandTest extends Command {

  private testLauncherPID_1 launcher;
  private double velocity;

  public LauncherPIDCommandTest(testLauncherPID_1 m_TestLauncherPID_1, Double velocity) {
    this.launcher = m_TestLauncherPID_1;
    this.velocity = velocity;
    addRequirements(m_TestLauncherPID_1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
