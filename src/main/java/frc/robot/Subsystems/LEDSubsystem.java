// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
/*     DigitalOutput port0 = new DigitalOutput(0);
    DigitalOutput port1 = new DigitalOutput(0);
    DigitalOutput port2 = new DigitalOutput(0); */
  }

  /*      LED combinations
   *  0.- 0, 0, 0 = 
   *  1.- 1, 0, 0 = 
   *  2.- 0, 1, 0 = 
   *  3.- 0, 0, 1 =   
   *  4.- 1, 1, 0 = 
   *  5.- 1, 0, 1 = 
   *  6.- 0, 1, 1 = 
   *  7.- 1, 1, 1 =  
   */

  public void combinationAssigner(int combination){
    switch(combination){
      case 0:
      
      break;
      case 1:

      break;
      case 2:

      break;
      case 3:

      break;
      case 4:

      break;
      case 5:

      break;
      case 6:

      break;
      case 7:
      break;
    }
  }

  public void pinWriter(int pin0, int pin1, int pin2){
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
