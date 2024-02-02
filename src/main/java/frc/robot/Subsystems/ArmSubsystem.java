// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm_leftMotor = new CANSparkMax(ArmConstants.arm_leftMotor_PORT, MotorType.kBrushless);
  private CANSparkMax arm_rightMotor = new CANSparkMax(ArmConstants.arm_rightMotor_PORT,
      MotorType.kBrushless); // LeftMotor = 14, RightMotor = 31

  private RelativeEncoder arm_Encoder;
  PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
  private double processVar;
  private double Setpoint;

  public ArmSubsystem() {
    arm_Encoder = arm_rightMotor.getEncoder();
    arm_leftMotor.follow(arm_rightMotor);
  }

  public Command set(double Output) {
    return this.run(() -> arm_rightMotor.set(Output));
  }

  public Command setSetpoint(double Setpoint) {
    this.Setpoint = Setpoint;
    return runOnce(() -> pid.setSetpoint(Setpoint));
  }

  public Command setSetpointManual(BooleanSupplier povLeft, BooleanSupplier povRight) {
    if (povLeft.getAsBoolean() == true) {
      Setpoint = Setpoint -1;
    }
    if (povRight.getAsBoolean() == true) {
      Setpoint = Setpoint +1;
    }
    
    return runOnce(() -> pid.setSetpoint(Setpoint));
  }

  @Override
  public void periodic() {
    super.periodic();
    processVar = pid.calculate(arm_Encoder.getPosition());
    arm_rightMotor.set(processVar * 0.2);

    SmartDashboard.putNumber("Setpoint: ", pid.getSetpoint());
    SmartDashboard.putNumber("Encoder: ", arm_Encoder.getPosition());
    SmartDashboard.putNumber("Process Variable: ", processVar);
    SmartDashboard.putNumber("Error ", pid.getPositionError());
  }
}
