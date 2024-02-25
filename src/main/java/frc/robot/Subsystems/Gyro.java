// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class Gyro extends SubsystemBase {
  ADIS16470_IMU Gyro = new ADIS16470_IMU();
  public Gyro() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double getZ(){

    return Gyro.getAngle(IMUAxis.kYaw);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void periodic(){
  }
}
