// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class DriveWithJoystick extends Command {
  /** Creates a new DriveWithJoystick. */

  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier joystickX;
  private final DoubleSupplier joystickZ;
  private final DoubleSupplier rightTrigger;
  private final DoubleSupplier leftTrigger;
  private final boolean buttonToggle;

  public DriveWithJoystick(DrivetrainSubsystem drivetrain, DoubleSupplier joystickX, 
      DoubleSupplier joystickZ, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger, boolean buttonToggle ) {
    this.drivetrain = drivetrain;
    this.joystickX = joystickX;
    this.joystickZ = joystickZ;
    this.rightTrigger = rightTrigger;
    this.leftTrigger = leftTrigger;
    this.buttonToggle = buttonToggle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Boolean Inverted = false;
    
    drivetrain.drive(joystickX, joystickZ, rightTrigger, leftTrigger, buttonToggle);
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    drivetrain.drive(joystickX, joystickZ, rightTrigger, leftTrigger, buttonToggle);

    //drivetrain.drive(joystickX, joystickY, joystickZ);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
