// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);

  PIDController pid_rightRear = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
      DrivetrainConstants.kD);
  PIDController pid_leftRear = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
      DrivetrainConstants.kD);

  private double processVar_rightRear;
  private double processVar_leftRear;

  private ADIS16470_IMU Gyroscope = new ADIS16470_IMU();

  MecanumDrive m_drive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set,
      rightRearMotor::set);

  // private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  // private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder rightRearEncoder = rightRearMotor.getEncoder();
  private RelativeEncoder leftRearEncoder = leftRearMotor.getEncoder();
  // Este encoder leftRear da negativo

  public DrivetrainSubsystem() {
    rightRearMotor.setInverted(true);
    rightFrontMotor.setInverted(true);

    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);

    leftRearEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor); // esto usa rotaciones y  // se multiplica por el // argumento
    rightRearEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    leftRearEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);
    rightRearEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);

    Gyroscope.calibrate();
  }

  // Encoders
  public void resetEncoders() {
    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);
  }

  public void resetEncoder_rightRear() {
    rightRearEncoder.setPosition(0);
  }

  public void resetEncoder_leftRear() {
    leftRearEncoder.setPosition(0);
  }

  public void resetGyro() {
    Gyroscope.reset();
  }

  public double getRightEncoderPosition() {
    return rightRearEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftRearEncoder.getPosition();
  }

  public double getAvarageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2);
  }

  public Command calculatePID_rightRear(double Setpoint) {
    return run(() -> {
      boolean stop = false;
      pid_rightRear.setSetpoint(Setpoint);
      while (stop == true) {
        processVar_rightRear = pid_rightRear.calculate(rightRearEncoder.getPosition());
        rightRearMotor.set(processVar_rightRear * 0.7);
        rightFrontMotor.set(processVar_rightRear * 0.7);
        if ((rightRearEncoder.getPosition() > Setpoint - 2) && (rightRearEncoder.getPosition() < Setpoint + 2)) {
          stop = true;
        }
      }
      resetEncoder_rightRear();
    });

  }
   

  public Command calculatePID_leftRear(double Setpoint) {
    return run(() -> {
      boolean stop = false;
      pid_leftRear.setSetpoint(Setpoint);
      while (stop) {
        processVar_leftRear = pid_leftRear.calculate(leftRearEncoder.getPosition());
        leftRearMotor.set(processVar_leftRear * 0.7);
        leftFrontMotor.set(processVar_leftRear * 0.7);
        if ((leftRearEncoder.getPosition() > Setpoint - 2) && (leftRearEncoder.getPosition() < Setpoint + 2)) {
          stop = true;
        }
      }
      resetEncoder_leftRear();
    });

  }


  public Command calculatePID_drive(double Setpoint_rightRear, double Setpoint_leftRear, double speed) {
    return runOnce(() -> {

      resetEncoders();
      pid_rightRear.setSetpoint(Setpoint_rightRear);
      pid_leftRear.setSetpoint(Setpoint_leftRear);
    }) // Este encoder leftRear da negativo el setpoint debe ser negativo
        .andThen(run(() -> {
          processVar_rightRear = pid_rightRear.calculate(rightRearEncoder.getPosition());
          processVar_leftRear = pid_leftRear.calculate(leftRearEncoder.getPosition());

          rightRearMotor.set(processVar_rightRear * speed); // este esta invertido
          rightFrontMotor.set(processVar_rightRear * speed); // cuando le meto este se invierte wtf

          leftRearMotor.set(processVar_leftRear * speed); // este esta invertido
          leftFrontMotor.set(processVar_leftRear * speed);
        }))
        .until(() -> ((Math.abs(rightRearEncoder.getPosition()) >= Math.abs(Setpoint_rightRear * 0.95))
            && (Math.abs(leftRearEncoder.getPosition()) >= Math.abs(-Setpoint_leftRear * 0.95))))
        .finallyDo(() -> {
          rightRearMotor.set(0);
          rightFrontMotor.set(0);
          leftRearMotor.set(0); 
          leftFrontMotor.set(0);
        });

  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Posicion atras izquierdo: ", getLeftEncoderPosition());
    SmartDashboard.putNumber("Posicion atras derecho: ", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyroscope", Gyroscope.getAngle(IMUAxis.kYaw));
  }

  public Command driveTest(){
    return run(() ->{
      m_drive.driveCartesian(0, 0.2, 0);
    });
  }


  public void drive(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
  DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {

        if (rightTrigger.getAsDouble() > 0.1) {
          if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
              Math.abs(joystickY.getAsDouble()) < 0.1 && 
              Math.abs(joystickZ.getAsDouble()) < 0.1 && 
              Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
              Math.abs(leftTrigger.getAsDouble()) < 0.1) {

              m_drive.driveCartesian(0, 0, 0);

            } 
            else
            {
              m_drive.driveCartesian(rightTrigger.getAsDouble(), 
                                    -joystickX.getAsDouble(), 
                                    joystickZ.getAsDouble());
            }
        } else {
          if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
              Math.abs(joystickY.getAsDouble()) < 0.1 && 
              Math.abs(joystickZ.getAsDouble()) < 0.1 &&
              Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
              Math.abs(leftTrigger.getAsDouble()) < 0.1) {
            m_drive.driveCartesian(0, 0, 0);
          } else {
            m_drive.driveCartesian(-leftTrigger.getAsDouble(),
                                   -joystickX.getAsDouble(), 
                                   joystickZ.getAsDouble());
          }
        }
}


  public void driveInverted(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
    DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {

      if (rightTrigger.getAsDouble() > 0.1) {
        if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
            Math.abs(joystickY.getAsDouble()) < 0.1 && 
            Math.abs(joystickZ.getAsDouble()) < 0.1 && 
            Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
            Math.abs(leftTrigger.getAsDouble()) < 0.1) {

            m_drive.driveCartesian(0, 0, 0);

          } 
          else
          {
            m_drive.driveCartesian(joystickY.getAsDouble(), 
                                   rightTrigger.getAsDouble(), 
                                   joystickZ.getAsDouble());
          }
      } else {
        if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
            Math.abs(joystickY.getAsDouble()) < 0.1 && 
            Math.abs(joystickZ.getAsDouble()) < 0.1 &&
            Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
            Math.abs(leftTrigger.getAsDouble()) < 0.1) {
          m_drive.driveCartesian(0, 0, 0);
        } else {
          m_drive.driveCartesian(joystickY.getAsDouble(), 
                                 -leftTrigger.getAsDouble(),
                                 joystickZ.getAsDouble());
        }
      }
}


  public Command driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
        // Reset encoders at the start of the command
        () -> resetEncoders())
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.driveCartesian(0, speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.max(getLeftEncoderPosition(), getRightEncoderPosition()) >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());

  }
  
  public Command autoTurnLeft(Double angle) {
    return 
    runOnce(() -> resetGyro()).andThen(() ->    
    run(() -> {
    if (Gyroscope.getAngle(IMUAxis.kYaw) > -angle - 4){
      m_drive.driveCartesian(0, 0, (0.15 + ((Gyroscope.getAngle(IMUAxis.kYaw) + (Gyroscope.getAngle(IMUAxis.kYaw) - angle)) * 0.001)));
    } 
    else if (Gyroscope.getAngle(IMUAxis.kYaw) < -angle + 4) {
      m_drive.driveCartesian(0, 0, -(0.15 + ((Gyroscope.getAngle(IMUAxis.kYaw) - (Gyroscope.getAngle(IMUAxis.kYaw) - angle)) * 0.001)));
    }
  }).until(() -> (Gyroscope.getAngle(IMUAxis.kYaw) < angle - 4) && (Gyroscope.getAngle(IMUAxis.kYaw) > angle + 4))).finallyDo(interrupted -> m_drive.stopMotor());
  }
  
  public Command autoTurnRight(Double angle) {
    return 
    runOnce(() -> resetGyro()).andThen(() ->     
    run(() -> {
    if (Gyroscope.getAngle(IMUAxis.kYaw) < angle - 4){
      m_drive.driveCartesian(0, 0, -(0.15 + ((Gyroscope.getAngle(IMUAxis.kYaw) + (Gyroscope.getAngle(IMUAxis.kYaw) - angle)) * 0.001)));
    } 
    else if (Gyroscope.getAngle(IMUAxis.kYaw) > angle + 4) {
      m_drive.driveCartesian(0, 0, 0.15 + ((Gyroscope.getAngle(IMUAxis.kYaw) - (Gyroscope.getAngle(IMUAxis.kYaw) - angle)) * 0.001));
    }
  }).until(() -> (Gyroscope.getAngle(IMUAxis.kYaw) > angle - 4) && (Gyroscope.getAngle(IMUAxis.kYaw) < angle + 4))).finallyDo(interrupted -> m_drive.stopMotor());
  }


}
