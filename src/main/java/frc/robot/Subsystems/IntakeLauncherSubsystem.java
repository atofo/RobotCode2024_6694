// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeLauncherConstants;

public class IntakeLauncherSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_intakeMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);

  private DigitalInput intakeSwitch = new DigitalInput(IntakeLauncherConstants.intakelauncher_intakeSwitch_PORT);

  /** Creates a new IntakeLauncher. */
  public IntakeLauncherSubsystem() {

  }

  public void setEverythingOFF(){
    m_intakeMotor.set(0);

    new WaitCommand(3);

    m_downMotor.set(0);
    m_upMotor.set(0);
   
  }

  public void prepareNote(){
    while(!intakeSwitch.get()){
      m_intakeMotor.set(-0.8);
      }
      while(intakeSwitch.get()){
        m_intakeMotor.set(-0.1);
        }
      while(!intakeSwitch.get()){
        m_intakeMotor.set(0);
      }

  new WaitCommand(3);

   
  }

  
  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean("Limit switrch", intakeSwitch.get());
    // This method will be called once per scheduler run
  }
}
