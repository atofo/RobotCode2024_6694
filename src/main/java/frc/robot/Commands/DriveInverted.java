// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class DriveInverted extends Command {
  /** Creates a new DriveWithJoystick. */

  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier joystickX;
  private final DoubleSupplier joystickY;
  private final DoubleSupplier joystickZ;
  private final DoubleSupplier rightTrigger;
  private final DoubleSupplier leftTrigger;

  public DriveInverted(DrivetrainSubsystem drivetrain, DoubleSupplier joystickX, DoubleSupplier joystickY,
      DoubleSupplier joystickZ, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
    this.drivetrain = drivetrain;
    this.joystickX = joystickX;
    this.joystickY = joystickY;
    this.joystickZ = joystickZ;
    this.rightTrigger = rightTrigger;
    this.leftTrigger = leftTrigger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Boolean Inverted = false;
    
    drivetrain.driveInverted(joystickX, joystickY, joystickZ, rightTrigger, leftTrigger);
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    drivetrain.driveInverted(joystickX, joystickY, joystickZ, rightTrigger, leftTrigger);
  
    //drivetrain.drive(joystickX, joystickY, joystickZ);
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
