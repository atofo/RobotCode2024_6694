// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class DriveWithJoystick extends Command {
  /** Creates a new DriveWithJoystick. */

  private final Drivetrain drivetrain;
  private final XboxController driverController;

  public DriveWithJoystick(Drivetrain drivetrain, XboxController driverController) {
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Boolean Inverted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickX = driverController.getRawAxis(1);
    double joystickY = driverController.getRawAxis(0);
    double joystickZ = driverController.getRawAxis(4);

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
