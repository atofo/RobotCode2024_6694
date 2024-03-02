// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private SparkPIDController m_downPidController = m_downMotor.getPIDController();
  private RelativeEncoder m_downEncoder = m_downMotor.getEncoder();
  private double downSetPoint = ShooterConstants.maxRPM;
  
  
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);
  private SparkPIDController m_upPidController = m_upMotor.getPIDController();
  private RelativeEncoder m_upEncoder = m_upMotor.getEncoder();
  private double upSetPoint = ShooterConstants.maxRPM;

  public ShooterSubsystem() {
    m_upMotor.setInverted(true);
    upSetPoint =  0;
    downSetPoint = 0;

    m_upPidController.setP(ShooterConstants.kP);
    m_upPidController.setI(ShooterConstants.kI);
    m_upPidController.setD(ShooterConstants.kD);
    m_upPidController.setIZone(ShooterConstants.kIz);
    m_upPidController.setFF(ShooterConstants.kFF);
    m_upPidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    
    m_downPidController.setP(ShooterConstants.kP);
    m_downPidController.setI(ShooterConstants.kI);
    m_downPidController.setD(ShooterConstants.kD);
    m_downPidController.setIZone(ShooterConstants.kIz);
    m_downPidController.setFF(ShooterConstants.kFF);
    m_downPidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }

  public void enable() {
      upSetPoint = (ShooterConstants.maxRPM);
      downSetPoint = (ShooterConstants.maxRPM);
  }
  
  public void disable() {
      upSetPoint = (0);
      downSetPoint = (0);
  }

  public boolean atSetpoint() {
    if(((m_upEncoder.getVelocity() > ShooterConstants.maxRPM - ShooterConstants.RPMmargin) && (m_upEncoder.getVelocity() < ShooterConstants.maxRPM + ShooterConstants.RPMmargin))
    && ((m_downEncoder.getVelocity() > ShooterConstants.maxRPM - ShooterConstants.RPMmargin) && (m_downEncoder.getVelocity() < ShooterConstants.maxRPM + ShooterConstants.RPMmargin))){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic(){
    super.periodic();
    m_upPidController.setReference(upSetPoint, CANSparkMax.ControlType.kVelocity);
    m_downPidController.setReference(downSetPoint, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("SetPoint", upSetPoint);
    SmartDashboard.putNumber("Up Velocity", m_upEncoder.getVelocity());
    SmartDashboard.putNumber("Down Velocity", m_downEncoder.getVelocity());
    SmartDashboard.putBoolean("atSetpoint", atSetpoint());
    
  }

}
