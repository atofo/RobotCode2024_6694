// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeLauncherConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_intakeMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_intakeMotor_PORT, MotorType.kBrushless);
  private DigitalInput intakeSwitch = new DigitalInput(IntakeLauncherConstants.intakelauncher_intakeSwitch_PORT);
  private int state=0;

  public IntakeSubsystem() {}

  public void getNote(){
    if(intakeSwitch.get()){
      if(state == 0){
        state++;
      }
    }
    else{
      if(state == 1){
        state++;
      }
    }

    switch(state){
      case 0:
      m_intakeMotor.set(-0.8);

      break;

      case 1:
      m_intakeMotor.set(-0.1);

      break;

      case 2:
      m_intakeMotor.set(0);

      break;
    }
    }
    
  public void intakeOFF(){
        m_intakeMotor.set(0);
        state = 0;
      }

  public void throwNote(){
    m_intakeMotor.set(-1);
  }

/*   @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean("Switch", intakeSwitch.get());

  } */
}
