// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AutoDriveApril;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class DriveAuto extends Command {

  AutoDriveApril driveAuto;
  DrivetrainSubsystem driveTrain;



  public DriveAuto(DrivetrainSubsystem driveTrain, AutoDriveApril driveAuto) {

    this.driveTrain = driveTrain;
    this.driveAuto = driveAuto;

    addRequirements(driveAuto);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveTrain.drive(() -> 0, () -> 0, () -> 0);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrain.drive(() -> driveAuto.XSpeed(() -> 0), () -> driveAuto.YSpeed(() -> 0.50), () -> driveAuto.ZSpeed(() -> 0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveTrain.leftFrontMotor.stopMotor();
    driveTrain.leftRearMotor.stopMotor();
    driveTrain.rightFrontMotor.stopMotor();
    driveTrain.rightFrontMotor.stopMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
} */