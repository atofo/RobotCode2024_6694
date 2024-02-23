// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT,
      MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);

  private ADIS16470_IMU Gyroscope = new ADIS16470_IMU();

  MecanumDrive m_drive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set,
      rightRearMotor::set);

 

  public DrivetrainSubsystem() {
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    Gyroscope.calibrate();

    
  }

  public void drive(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
      DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {

    if (rightTrigger.getAsDouble() > 0.1) {
      if (Math.abs(joystickX.getAsDouble()) < 0.1 && Math.abs(joystickY.getAsDouble()) < 0.1
          && Math.abs(joystickZ.getAsDouble()) < 0.1 && Math.abs(rightTrigger.getAsDouble()) < 0.1
          && Math.abs(leftTrigger.getAsDouble()) < 0.1) {
        m_drive.driveCartesian(0, 0, 0);
      } else {
        m_drive.driveCartesian(rightTrigger.getAsDouble(), joystickY.getAsDouble(), joystickZ.getAsDouble());
      }
    }

    else {
      if (Math.abs(joystickX.getAsDouble()) < 0.1 && Math.abs(joystickY.getAsDouble()) < 0.1
          && Math.abs(joystickZ.getAsDouble()) < 0.1 && Math.abs(rightTrigger.getAsDouble()) < 0.1
          && Math.abs(leftTrigger.getAsDouble()) < 0.1) {
        m_drive.driveCartesian(0, 0, 0);
      } else {
        m_drive.driveCartesian(-leftTrigger.getAsDouble(), joystickY.getAsDouble(), joystickZ.getAsDouble());
      }
    }

  }


/*   @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Gyroscope", Gyroscope.getAngle(IMUAxis.kYaw));
  } */
}
