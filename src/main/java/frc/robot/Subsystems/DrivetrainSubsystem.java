// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class DrivetrainSubsystem extends SubsystemBase {

  private final PhotonCamera Limelight;
  private int Id;
  private double yaw;
  private double pitch;
  private double Area;
  private double skew;
  private boolean inverted = false;

  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);

  PIDController pid_rightRear = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
      DrivetrainConstants.kD);
  PIDController pid_leftRear = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
      DrivetrainConstants.kD);
  PIDController pid_rightFront = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
      DrivetrainConstants.kD);
  PIDController pid_leftFront = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
      DrivetrainConstants.kD);

  private double processVar_rightRear;
  private double processVar_leftRear;
  private double processVar_rightFront;
  private double processVar_leftFront;

  private ADIS16470_IMU Gyroscope = new ADIS16470_IMU();

  MecanumDrive m_drive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set,
      rightRearMotor::set);

  private RelativeEncoder rightRearEncoder = rightRearMotor.getEncoder();
  private RelativeEncoder leftRearEncoder = leftRearMotor.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();

  private PhotonTrackedTarget target;
  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;

  public DrivetrainSubsystem() {

    rightRearMotor.setInverted(true);
    rightFrontMotor.setInverted(true);

    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);

    leftRearEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor); // esto usa rotaciones y
                                                                                               // // se multiplica por
                                                                                               // el // argumento
    rightRearEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);

    leftRearEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);
    rightRearEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);
    leftFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);
    rightFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);

    Limelight = new PhotonCamera("photonvision");
    limelightZero();

    Gyroscope.calibrate();

    leftFrontMotor.enableVoltageCompensation(11.7);
    rightFrontMotor.enableVoltageCompensation(11.7);
    leftRearMotor.enableVoltageCompensation(11.7);
    rightRearMotor.enableVoltageCompensation(11.7);
  }

  // Encoders
  public void resetEncoders() {
    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
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

  public double getRearRightEncoderPosition() {
    return rightRearEncoder.getPosition();
  }

  public double getRearLeftEncoderPosition() {
    return leftRearEncoder.getPosition();
  }
  public double getFrontRightEncoderPosition() {
    return rightFrontEncoder.getPosition();
  }

  public double getFrontLeftEncoderPosition() {
    return leftFrontEncoder.getPosition();
  }

  public double getAvarageEncoderDistance() {
    return ((getRearLeftEncoderPosition() + getRearRightEncoderPosition()) / 2);
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

  public Command calculatePID_drive(double Setpoint_rightRear, double Setpoint_leftRear, double speed, double timeout) {
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
        })).withTimeout(timeout)
        .until(() -> ((Math.abs(rightRearEncoder.getPosition()) >= Math.abs(Setpoint_rightRear * 0.95))
            && (Math.abs(leftRearEncoder.getPosition()) >= Math.abs(Setpoint_leftRear * 0.95))))
        .finallyDo(() -> {
          rightRearMotor.set(0);
          rightFrontMotor.set(0);
          leftRearMotor.set(0);
          leftFrontMotor.set(0);
        });
  }

  public Command FourNote_calculatePID_drive(double Setpoint_rightRear, double Setpoint_leftRear,
      double Setpoint_rightFront, double Setpoint_leftFront, double speed, double timeout) {
    return runOnce(() -> {
      resetEncoders();
      pid_rightRear.setSetpoint(Setpoint_rightRear);
      pid_leftRear.setSetpoint(Setpoint_leftRear);
      pid_rightFront.setSetpoint(Setpoint_rightFront);
      pid_leftFront.setSetpoint(Setpoint_leftFront);
    }) // Este encoder leftRear da negativo el setpoint debe ser negativo
        .andThen(run(() -> {
          processVar_rightRear = pid_rightRear.calculate(rightRearEncoder.getPosition());
          processVar_leftRear = pid_leftRear.calculate(leftRearEncoder.getPosition());
          processVar_rightFront = pid_rightFront.calculate(rightFrontEncoder.getPosition());
          processVar_leftFront = pid_leftFront.calculate(leftFrontEncoder.getPosition());

          rightRearMotor.set(processVar_rightRear * speed); // este esta invertido
          rightFrontMotor.set(processVar_rightFront * speed); 

          leftRearMotor.set(processVar_leftRear * speed); // este esta invertido
          leftFrontMotor.set(processVar_leftFront * speed);
        })).withTimeout(timeout)
        .until(() -> ((Math.abs(rightRearEncoder.getPosition()) >= Math.abs(Setpoint_rightRear * 0.95))
        && (Math.abs(leftRearEncoder.getPosition()) >= Math.abs(Setpoint_leftRear * 0.95))
        && (Math.abs(rightFrontEncoder.getPosition()) >= Math.abs(Setpoint_rightFront * 0.95))
        && (Math.abs(leftFrontEncoder.getPosition()) >= Math.abs(Setpoint_leftFront * 0.95))))
        .finallyDo(() -> {
          rightRearMotor.set(0);
          rightFrontMotor.set(0);
          leftRearMotor.set(0);
          leftFrontMotor.set(0);
        });
  }

  public Command calculatePID_mecanumdrive(double Setpoint_leftDiagonal, double Setpoint_rightDiagonal, double speed,
      double timeout) {
    return runOnce(() -> {
      resetEncoders();
      pid_rightRear.setSetpoint(Setpoint_rightDiagonal);
      pid_leftRear.setSetpoint(Setpoint_leftDiagonal);
    })
        .andThen(run(() -> {
          processVar_rightRear = pid_rightRear.calculate(rightRearEncoder.getPosition());
          processVar_leftRear = pid_leftRear.calculate(leftRearEncoder.getPosition());

          rightRearMotor.set(processVar_rightRear * speed);
          leftFrontMotor.set(processVar_rightRear * speed);

          rightFrontMotor.set(processVar_leftRear * speed);
          leftRearMotor.set(processVar_leftRear * speed);
        })).withTimeout(timeout)
        .until(() -> ((Math.abs(rightRearEncoder.getPosition()) >= Math.abs(Setpoint_rightDiagonal * 0.95))
            && (Math.abs(leftRearEncoder.getPosition()) >= Math.abs(Setpoint_leftDiagonal * 0.95))))
        .finallyDo(() -> {
          rightRearMotor.set(0);
          rightFrontMotor.set(0);
          leftRearMotor.set(0);
          leftFrontMotor.set(0);
        });
  }

  public Command calculatePID_cleandrive(double Setpoint_leftDiagonal, double Setpoint_rightDiagonal, double speedrR,
      double speedrF, double speed, double timeout) {
    return runOnce(() -> {
      resetEncoders();
      pid_rightRear.setSetpoint(Setpoint_rightDiagonal);
      pid_leftRear.setSetpoint(Setpoint_leftDiagonal);
    })
        .andThen(run(() -> {
          processVar_rightRear = pid_rightRear.calculate(rightRearEncoder.getPosition());
          processVar_leftRear = pid_leftRear.calculate(leftRearEncoder.getPosition());

          rightRearMotor.set(processVar_rightRear * speed);
          leftFrontMotor.set(processVar_rightRear * speed);

          rightFrontMotor.set(processVar_leftRear * speed);
          leftRearMotor.set(processVar_leftRear * speed);
        })).withTimeout(timeout)
        .until(() -> ((Math.abs(rightRearEncoder.getPosition()) >= Math.abs(Setpoint_rightDiagonal * 0.95))
            && (Math.abs(leftRearEncoder.getPosition()) >= Math.abs(Setpoint_leftDiagonal * 0.95))))
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
    SmartDashboard.putNumber("Posicion atras izquierdo: ", getRearLeftEncoderPosition());
    SmartDashboard.putNumber("Posicion atras derecho: ", getRearRightEncoderPosition());
    SmartDashboard.putNumber("Posicion enfrente izquierdo: ", getFrontLeftEncoderPosition());
    SmartDashboard.putNumber("Posicion enfrente derecho: ", getFrontRightEncoderPosition());
    SmartDashboard.putNumber("Gyroscope", Gyroscope.getAngle(IMUAxis.kYaw));
    var result = Limelight.getLatestResult();

    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.toArray().length > 0) {
      Area = targets.get(0).getArea();
      yaw = targets.get(0).getYaw();
      SmartDashboard.putNumber("April ID 0", targets.get(0).getFiducialId());

    }
  }

  public void drive(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
      DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {

    inverted = false;
    if (rightTrigger.getAsDouble() > 0.1) {
      if (Math.abs(joystickX.getAsDouble()) < 0.1 &&
          Math.abs(joystickY.getAsDouble()) < 0.1 &&
          Math.abs(joystickZ.getAsDouble()) < 0.1 &&
          Math.abs(rightTrigger.getAsDouble()) < 0.1 &&
          Math.abs(leftTrigger.getAsDouble()) < 0.1) {

        m_drive.driveCartesian(0, 0, 0);

      } else {
        m_drive.driveCartesian(rightTrigger.getAsDouble(),
            -joystickX.getAsDouble(),
            -joystickZ.getAsDouble());
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
            -joystickZ.getAsDouble());
      }
    }
  }

  public void driveInverted(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
      DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
    inverted = true;
    if (rightTrigger.getAsDouble() > 0.1) {
      if (Math.abs(joystickX.getAsDouble()) < 0.1 &&
          Math.abs(joystickY.getAsDouble()) < 0.1 &&
          Math.abs(joystickZ.getAsDouble()) < 0.1 &&
          Math.abs(rightTrigger.getAsDouble()) < 0.1 &&
          Math.abs(leftTrigger.getAsDouble()) < 0.1) {

        m_drive.driveCartesian(0, 0, 0);

      } else {
        m_drive.driveCartesian(-rightTrigger.getAsDouble(),
            joystickX.getAsDouble(),
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
        m_drive.driveCartesian(leftTrigger.getAsDouble(),
            joystickX.getAsDouble(),
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
            () -> Math.max(getRearLeftEncoderPosition(), getRearRightEncoderPosition()) >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());

  }

  /*
   * public Command StraightApril() {
   * 
   * return run(() -> m_drive.driveCartesian(0, 0, XSpeed(() -> 0.00))).until(()
   * -> XSpeed(() -> 0) == 0);
   * 
   * }
   */

  /*
   * public double XSpeed(DoubleSupplier DYaw) {
   * if (yaw > DYaw.getAsDouble() + 1) { // Rango de error para VelocidadX ((SOLO
   * CAMBIAR DYaw)
   * return -0.2;
   * } else if (yaw < DYaw.getAsDouble() - 1) {
   * return 0.2;
   * } else {
   * return 0;
   * }
   * }
   */

  public void limelightZero() {
    Id = 0;
    yaw = 0;
    pitch = 0;
    Area = 0;
    skew = 0;
  }

  public double limelightArea() {
    return Area;
  }

  public boolean inverted() {
    if (inverted) {
      return true;
    } else {
      return false;
    }
  }

}
