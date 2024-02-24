// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm_leftMotor = new CANSparkMax(ArmConstants.arm_leftMotor_PORT, MotorType.kBrushless);
  private CANSparkMax arm_rightMotor = new CANSparkMax(ArmConstants.arm_rightMotor_PORT, MotorType.kBrushless);

  private RelativeEncoder arm_Encoder = arm_rightMotor.getEncoder();

  PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

  private double processVar;
  private double Setpoint;

  public ArmSubsystem() {

    arm_leftMotor.follow(arm_rightMotor);
    pid.setSetpoint(-5);
  }


@Override
  public void periodic() {
    super.periodic();
    processVar = pid.calculate(arm_Encoder.getPosition());
    arm_rightMotor.set(processVar * 0.7);  
    arm_leftMotor.set(processVar * 0.7);

    SmartDashboard.putNumber("Setpoint: ", pid.getSetpoint());
    SmartDashboard.putNumber("Encoder: ", arm_Encoder.getPosition());
    SmartDashboard.putNumber("Process Variable: ", processVar);
    SmartDashboard.putNumber("Error ", pid.getPositionError());


  }

  public Command setSetpoint(double Setpoint) {
    // this.Setpoint = Setpoint;
    return runOnce(() -> pid.setSetpoint(Setpoint));
  }

   public void manualSetpointDown() {
      Setpoint = pid.getSetpoint() + 1;
      pid.setSetpoint(Setpoint);
    }
   public void manualSetpointUp() {
      Setpoint = pid.getSetpoint() - 1;
      pid.setSetpoint(Setpoint);
    }

   public void setpointStop() {
      pid.setSetpoint(arm_Encoder.getPosition());
    }
   
   }
