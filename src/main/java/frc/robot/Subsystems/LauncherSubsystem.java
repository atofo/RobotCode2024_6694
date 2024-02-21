// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeLauncherConstants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);
  
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
  );
 */
public LauncherSubsystem() {
    m_upMotor.restoreFactoryDefaults();
    m_downMotor.restoreFactoryDefaults();
    
    m_upMotor.burnFlash();
    m_downMotor.burnFlash();

    m_upMotor.setInverted(true);
  }

  public void throwNote(){
    m_downMotor.set(1);
    m_upMotor.set(1);
  }

  public void launcherOFF(){
      m_downMotor.set(0);
      m_upMotor.set(0);
  }
/* 
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction); 
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
