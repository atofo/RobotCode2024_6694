// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT,
      MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);

  MecanumDrive m_drive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set,
      rightRearMotor::set);

  private final SysIdRoutine m_sysIdRoutine = 
  new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (volts) -> {
        leftFrontMotor.setVoltage(volts.in(Volts));
        rightFrontMotor.setVoltage(volts.in(Volts));
        leftRearMotor.setVoltage(volts.in(Volts));
        rightRearMotor.setVoltage(volts.in(Volts));
      },
      null, // No log consumer, since data is recorded by URCL
      this
    )
  );

  public DrivetrainSubsystem() {

    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
