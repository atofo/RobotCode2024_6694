// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.Constants.PIDConstants_Launcher;

public class testLauncherPID_1 extends SubsystemBase {

  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);

  private RelativeEncoder m_downMotorEncoder = m_downMotor.getEncoder();
  private RelativeEncoder m_upMotorEncoder = m_upMotor.getEncoder();

  private SparkPIDController m_downMotorPIDController;

  private double targetVelocity;



  public testLauncherPID_1() {
    m_upMotor.follow(m_downMotor);
    m_upMotor.setInverted(true);

    m_downMotorPIDController = m_downMotor.getPIDController();

    m_downMotorPIDController.setP(PIDConstants_Launcher.kP);
    m_downMotorPIDController.setI(PIDConstants_Launcher.kI);
    m_downMotorPIDController.setD(PIDConstants_Launcher.kD);
    m_downMotorPIDController.setIZone(PIDConstants_Launcher.kI);
    m_downMotorPIDController.setFF(PIDConstants_Launcher.kFF_downMotor);
    m_downMotorPIDController.setOutputRange(PIDConstants_Launcher.minPIDOutput, PIDConstants_Launcher.maxPIDOutput);

    m_downMotor.burnFlash();
    m_upMotor.burnFlash();
  }

  public void setVelocity(double velocity){
    targetVelocity = velocity;
    m_downMotorPIDController.setReference(targetVelocity, ControlType.kVelocity);
  }

  public void setSpeed(double speed){
      m_downMotor.set(speed);
    }
  
  public double getVelocity(){
    double sum = m_downMotorEncoder.getVelocity() + m_upMotorEncoder.getVelocity();
    double avg = sum / 2;
    return avg;
  }

  public void stop(){
    setSpeed(0.0);
  }



  @Override
  public void periodic() {
    
  

    SmartDashboard.putNumber("Down Velocity", m_downMotorEncoder.getVelocity());
    SmartDashboard.putNumber("UpVelocity", m_upMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Average Velocity", getVelocity());
    SmartDashboard.putNumber("Target Velocity", targetVelocity);
    
  }
}
