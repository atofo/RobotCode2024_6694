// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class DrivetrainSubsystem extends SubsystemBase {

  // Invert Drive
  private boolean toggleDrive = false;
  

  // Drive Forward
  private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;
  public double getEncoderMeters(){
    return (leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition()) / 2 *kEncoderTick2Meter;
  }

  // Motors
  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT,MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);


  // Encoders
  private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder leftRearEncoder = leftRearMotor.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder rightRearEncoder = rightRearMotor.getEncoder();


  // Gyro
  private ADIS16470_IMU Gyroscope = new ADIS16470_IMU();

  // Drive w joystick
  MecanumDrive m_drive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set, rightRearMotor::set);

 /*  private final SysIdRoutine m_sysIdRoutine = 
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
  ); */

  public DrivetrainSubsystem() {

    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    rightFrontMotor.restoreFactoryDefaults();
    rightRearMotor.restoreFactoryDefaults();
    leftFrontMotor.restoreFactoryDefaults();
    leftRearMotor.restoreFactoryDefaults();

    rightFrontMotor.burnFlash();
    rightRearMotor.burnFlash();
    leftFrontMotor.burnFlash();
    leftRearMotor.burnFlash();

    Gyroscope.calibrate();
    
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
                  m_drive.driveCartesian(joystickZ.getAsDouble(), 
                                         joystickY.getAsDouble(), 
                                         rightTrigger.getAsDouble());
                }
            } else {
              if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
                  Math.abs(joystickY.getAsDouble()) < 0.1 && 
                  Math.abs(joystickZ.getAsDouble()) < 0.1 &&
                  Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
                  Math.abs(leftTrigger.getAsDouble()) < 0.1) {
                m_drive.driveCartesian(0, 0, 0);
              } else {
                m_drive.driveCartesian(joystickZ.getAsDouble(), 
                                       joystickY.getAsDouble(),
                                      -leftTrigger.getAsDouble());
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
                m_drive.driveCartesian(-joystickZ.getAsDouble(), 
                                       -joystickY.getAsDouble(), 
                                       -rightTrigger.getAsDouble());
              }
          } else {
            if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
                Math.abs(joystickY.getAsDouble()) < 0.1 && 
                Math.abs(joystickZ.getAsDouble()) < 0.1 &&
                Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
                Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(-joystickZ.getAsDouble(), 
                                     -joystickY.getAsDouble(),
                                      leftTrigger.getAsDouble());
            }
          }
  }
  



  /* public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  } */

  @Override
  public void periodic() {
    super.periodic();

    // drivetrain encoder pos
    SmartDashboard.putNumber( "left front", leftFrontEncoder.getPosition());
    SmartDashboard.putNumber( "left rear", leftRearEncoder.getPosition());
    SmartDashboard.putNumber( "right front", rightFrontEncoder.getPosition());
    SmartDashboard.putNumber( "right rear", rightRearEncoder.getPosition());


    SmartDashboard.putNumber("Gyroscope", Gyroscope.getAngle(IMUAxis.kYaw));
  }
}