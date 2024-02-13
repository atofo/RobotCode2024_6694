// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.Constants.PIDConstants_Launcher;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);

  private PIDController pid = new PIDController(PIDConstants_Launcher.kP, PIDConstants_Launcher.kI, PIDConstants_Launcher.kD);

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(PIDConstants_Launcher.kS, PIDConstants_Launcher.kV, PIDConstants_Launcher.kA);

  private final Encoder launcherEncoder = new Encoder(2, 1);
  /* private final SysIdRoutine m_sysIdRoutine = 
  new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (volts) -> {
        m_downMotor.set(volts.in(Volts));
        m_upMotor.set(volts.in(Volts));
      },
      null, // No log consumer, since data is recorded by URCL
      this
    )
  ); */



  public LauncherSubsystem() {
    m_upMotor.setInverted(true);
    m_upMotor.follow(m_downMotor);
  }

  public void throwNote(){
    m_downMotor.set(1);
    m_upMotor.set(1);
  }

  public void launcherOFF(){
      m_downMotor.set(0);
      m_upMotor.set(0);
  }

  public void launcherWithPID(double VelocitySetpoint) {
    m_downMotor.setVoltage(feedforward.calculate(VelocitySetpoint)
        + pid.calculate(launcherEncoder.getRate(), VelocitySetpoint));

  }


 /*  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  } */

  @Override
  public void periodic() {
    super.periodic();
    m_downMotor.setVoltage( feedforward.calculate(100)
     + pid.calculate(launcherEncoder.getRate(), 100)); // Los setpoints deben de ser los mismos
  

    SmartDashboard.putNumber("FeedForward Output ", feedforward.calculate(100));
    SmartDashboard.putNumber("Pid Output ", pid.calculate(launcherEncoder.getRate(), 100));
    SmartDashboard.putNumber("Launcher encoder rate: ", launcherEncoder.getRate());
    SmartDashboard.putNumber("Launcher_motor applied output ", m_downMotor.getAppliedOutput());
    SmartDashboard.putNumber("Pid launcher get velocity error ", pid.getVelocityError());
    SmartDashboard.putNumber("Pid launcher get position error ", pid.getPositionError());
    
  }
}
