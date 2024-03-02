// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private DigitalOutput port0 = new DigitalOutput(LEDConstants.pin0);
  private DigitalOutput port1 = new DigitalOutput(LEDConstants.pin1);
  private DigitalOutput port2 = new DigitalOutput(LEDConstants.pin2);

  private int combination = 0;

  public LEDSubsystem() {}

  /*      LED combinations
  *  0.- 0, 0, 0 = Correr sin note dentro, FLAMAS ROJAS
  *  1.- 0, 0, 1 = Correr con note dentro, FLAMAS VERDES
  *  2.- 0, 1, 0 = Cuando el Shooter empiece a girar, MORADO ESTÁTICO
  *  3.- 0, 1, 1 = Cuando el Shooter este en sus RPM y el brazo este on point para tirar, FLAMAS VERDES
  *  4.- 1, 0, 0 = Cuando el drivetrain este invertido, FLAMAS AZULES
  *  5.- 1, 0, 1 = Brazo flamas rojas modo LOW, CLIMBER DERECHO VERDE ESTÁTICO
  *  6.- 1, 1, 0 = Brazo flamas rojas modo LOW, CLIMBER IZQUIERDO VERDE ESTÁTICO
  *  7.- 1, 1, 1 =  Brazo flamas rojas modo HIGH, AMBOS CLIMBERS FLAMA AZUL
  * 
  * 
  *  JERARQUIA
  *  7.- 1, 1, 1 =  Brazo flamas rojas modo HIGH, AMBOS CLIMBERS FLAMA AZUL               // PIXY SUBSYSTEM
  *  6.- 1, 1, 0 = Brazo flamas rojas modo LOW, CLIMBER IZQUIERDO VERDE ESTÁTICO          // PIXY SUBSYSTEM
  *  5.- 1, 0, 1 = Brazo flamas rojas modo LOW, CLIMBER DERECHO VERDE ESTÁTICO            // PIXY SUBSYSTEM
  *  3.- 0, 1, 1 = Cuando el Shooter este en sus RPM y el brazo este on point para tirar, FLAMAS VERDES   // SHOOTER Y ARM
  *  2.- 0, 1, 0 = Cuando el Shooter empiece a girar, MORADO ESTÁTICO                     // SHOOTER
  *  4.- 1, 0, 0 = Cuando el drivetrain este invertido, FLAMAS AZULES                     // DRIVETRAIN
  *  1.- 0, 0, 1 = Correr con note dentro, FLAMAS VERDES                                  // INTAKE
  *  0.- 0, 0, 0 = Correr sin note dentro, FLAMAS ROJAS                                   // INTAKE

   */

  public void combinationAssigner(int comb){
    combination = comb;
  }

  public void pinWriter(Boolean pin0, Boolean pin1, Boolean pin2){
    port0.set(pin0);
    port0.set(pin1);
    port0.set(pin2);
  }

  @Override
  public void periodic() {
    super.periodic();


    switch(combination){
      case 0:
      port0.set(false);
      port1.set(false);
      port2.set(false);

      break;
      case 1:
      port0.set(true);
      port1.set(false);
      port2.set(false);

      break;
      case 2:
      port0.set(false);
      port1.set(true);
      port2.set(false);

      break;
      case 3:
      port0.set(true);
      port1.set(true);
      port2.set(false);

      break;
      case 4:
      port0.set(false);
      port1.set(false);
      port2.set(true);

      break;
      case 5:
      port0.set(true);
      port1.set(false);
      port2.set(true);

      break;
      case 6:
      port0.set(false);
      port1.set(true);
      port2.set(true);

      break;
      case 7:
      port0.set(true);
      port1.set(true);
      port2.set(true);
      break;
    }
  }
}
