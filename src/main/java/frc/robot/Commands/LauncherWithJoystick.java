// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LauncherSubsystem;

public class LauncherWithJoystick extends Command {
  
  private LauncherSubsystem launcher;

  public LauncherWithJoystick(LauncherSubsystem m_LauncherSubsystem) {
    this.launcher = m_LauncherSubsystem;
     addRequirements(m_LauncherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  System.out.println("Shooter Initializeed");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        launcher.throwNote();
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooter End");
          launcher.launcherOFF();
  }
  


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
