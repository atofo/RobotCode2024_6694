// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.Constants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm_leftMotor = new CANSparkMax(ArmConstants.arm_leftMotor_PORT, MotorType.kBrushless);
  private CANSparkMax arm_rightMotor = new CANSparkMax(ArmConstants.arm_rightMotor_PORT,
      MotorType.kBrushless); // LeftMotor = 14, RightMotor = 31

  private final DutyCycleEncoder arm_Encoder = new DutyCycleEncoder(0);
  PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

  private double processVar;
  private double Setpoint;

  public ArmSubsystem() {
    arm_leftMotor.follow(arm_rightMotor);
    pid.setSetpoint(0.21);
  }

  /*
   * public Command set(double Output) {
   * return this.run(() -> arm_rightMotor.set(Output));
   * }
   */
  @Override
  public void periodic() {
    super.periodic();
    processVar = pid.calculate(arm_Encoder.getAbsolutePosition());
    arm_rightMotor.set(-processVar * 0.8);  
    arm_leftMotor.set(-processVar * 0.8);

    SmartDashboard.putNumber("Setpoint: ", pid.getSetpoint());
    SmartDashboard.putNumber("Encoder: ", arm_Encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Process Variable: ", processVar);
    SmartDashboard.putNumber("Error ", pid.getPositionError());

  }

  public Command setSetpoint(double Setpoint) {
    // this.Setpoint = Setpoint;
    return runOnce(() -> pid.setSetpoint(Setpoint));
  }

   public void manualSetpoint() {
      Setpoint += 0.001;
      pid.setSetpoint(Setpoint);
    }
   

  /*
   * public Command setSetpointManual(BooleanSupplier povLeft, BooleanSupplier
   * povRight) {
   * if (povLeft.getAsBoolean() == true) {
   * Setpoint = Setpoint -1;
   * }
   * if (povRight.getAsBoolean() == true) {
   * Setpoint = Setpoint +1;
   * }
   * 
   * return runOnce(() -> pid.setSetpoint(Setpoint));
   * }
   */
   }
