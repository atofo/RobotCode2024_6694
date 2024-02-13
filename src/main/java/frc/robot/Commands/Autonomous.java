// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class Autonomous extends Command {
  /** Creates a new Autonomous. */

  private final DrivetrainSubsystem m_drivetrain;
  private final Timer Timer = new Timer();
  ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  double AccelY;
  double TotalDisplacementY;
  double Displacement;
  double prevMecanumAccelY;

  
  public Autonomous(DrivetrainSubsystem drivetrain, DrivetrainSubsystem Timer) {
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gyro.reset();
    Timer.reset();
    Timer.start();
    TotalDisplacementY = 0;
    Displacement = 0;
    prevMecanumAccelY = 0.01;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Gyroscope", m_gyro.getAngle(IMUAxis.kYaw));
    SmartDashboard.putNumber("Accelerometer", AccelY);
    SmartDashboard.putNumber("Timer", Timer.get());

    AccelY = m_gyro.getAccelY();
    if (AccelY < 3) {
      AccelY = 0;
    } else if (AccelY > 3) {
      AccelY = AccelY;
    }
    
    Displacement = (0.5 * AccelY * (Timer.get()*Timer.get())) + TotalDisplacementY;
    TotalDisplacementY = Displacement;

    SmartDashboard.putNumber("Displacement", TotalDisplacementY);

    if (TotalDisplacementY < 200) {
      m_drivetrain.m_drive.driveCartesian(0.3, 0, 0);
    } else if (TotalDisplacementY >= 200) {
      m_drivetrain.m_drive.driveCartesian(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
