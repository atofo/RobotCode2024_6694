// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmPositionsWithJoystick extends Command {
  private final ArmSubsystem ArmSubsystem;
  private BooleanSupplier xButton;
  private BooleanSupplier bButton;
  private BooleanSupplier povRight;
  private BooleanSupplier povLeft;

  private double setpoint;

  /** Creates a new ArmPositionsWithJoystick. */
  public ArmPositionsWithJoystick(ArmSubsystem armSubsystem, BooleanSupplier xButton, BooleanSupplier bButton,
      BooleanSupplier povRight, BooleanSupplier povLeft) {
    this.ArmSubsystem = armSubsystem;
    this.xButton = xButton;
    this.bButton = bButton;
    this.povRight = povRight;
    this.povLeft = povLeft;

    addRequirements(ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xButton.getAsBoolean() == true) {
      setpoint = 5;
    }

    if (bButton.getAsBoolean() == true) {
      setpoint = -5;
    }

    if (povRight.getAsBoolean() == true) {
      setpoint = +1;
    }

    if (povLeft.getAsBoolean() == true) {
      setpoint = -1;
    }
    ArmSubsystem.setSetpoint(setpoint);
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
