// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/* import frc.robot.Commands.Arm_manualSetpointFront;
import frc.robot.Commands.Arm_manualSetpointBack; */
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.DriveInverted;
import frc.robot.Commands.DriveWithJoystick;
import frc.robot.Commands.Intake_getNote;
import frc.robot.Commands.Intake_returnNote;
import frc.robot.Commands.LauncherWithJoystick;
import frc.robot.Commands.LeftClimberDown;
import frc.robot.Commands.LeftClimberUp;
import frc.robot.Commands.RightClimberUp;
import frc.robot.Commands.RightClimberDown;
import frc.robot.Commands.Intake_ThrowNote;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;
import frc.robot.Subsystems.LeftClimber;
import frc.robot.Subsystems.RightClimber;

public class RobotContainer {
  //Sendabel Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  //Controllers
  private final CommandXboxController m_firstDriverController = new CommandXboxController(OperatorConstants.firstcontrollerPort);
  private final CommandXboxController m_secondDriverController = new CommandXboxController(OperatorConstants.secondcontrollerPort);


  //First Driver Triggers
  private Trigger L31 = m_firstDriverController.leftStick();
  private Trigger aButton1 = m_firstDriverController.a();

  private Trigger start1 = m_firstDriverController.start();
  private Trigger back1 = m_firstDriverController.back();
  
  private Trigger LB1 = m_firstDriverController.leftBumper();
  private Trigger RB1 = m_firstDriverController.rightBumper();
  
  
  //Second Driver Triggers
  private Trigger RT2 = m_secondDriverController.rightTrigger();
  private Trigger LT2 = m_secondDriverController.leftTrigger();
  
  private Trigger LB2 = m_secondDriverController.leftBumper();
  private Trigger RB2 = m_secondDriverController.rightBumper();
  
  private Trigger aButton2 = m_secondDriverController.a();
  private Trigger xButton2 = m_secondDriverController.x();
  private Trigger bButton2 = m_secondDriverController.b();
  private Trigger yButton2 = m_secondDriverController.y();


  private Trigger povRight2 = m_secondDriverController.povRight();
  private Trigger povLeft2 = m_secondDriverController.povLeft();
  private Trigger povDown2 = m_secondDriverController.povDown();
  private Trigger povUp2 = m_secondDriverController.povUp();
  
  private Trigger L32 = m_secondDriverController.leftStick();


  //Drivetrain
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final DriveWithJoystick m_DriveWithJoystick = new DriveWithJoystick(m_drivetrainSubsystem,
      () -> m_firstDriverController.getRawAxis(1), 
      () -> -m_firstDriverController.getRawAxis(0),
      () -> m_firstDriverController.getRawAxis(4), 
      () -> m_firstDriverController.getRawAxis(3),
      () -> m_firstDriverController.getRawAxis(2));

  private final DriveInverted m_DriveInverted = new DriveInverted(m_drivetrainSubsystem,   
   () -> -m_firstDriverController.getRawAxis(1), 
   () -> -m_firstDriverController.getRawAxis(0),
   () -> -m_firstDriverController.getRawAxis(4), 
   () -> m_firstDriverController.getRawAxis(3),
   () -> m_firstDriverController.getRawAxis(2));

  //Arm
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
/*   private final Arm_manualSetpointFront m_Arm_manualSetpointFront = new Arm_manualSetpointFront(m_ArmSubsystem);
  private final Arm_manualSetpointBack m_Arm_manualSetpointBack = new Arm_manualSetpointBack(m_ArmSubsystem);
 */
  //Intake
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final Intake_getNote m_Intake_getNote = new Intake_getNote(m_IntakeSubsystem);
  private final Intake_returnNote m_Intake_returnNote = new Intake_returnNote(m_IntakeSubsystem);
  private final Intake_ThrowNote m_Intake_throwNote = new Intake_ThrowNote(m_IntakeSubsystem);

  //Launcher
  private final LauncherSubsystem m_LauncherSubsystem = new LauncherSubsystem();
  private final LauncherWithJoystick m_LauncherWithJoystick = new LauncherWithJoystick(m_LauncherSubsystem);

  //Climbers
  private final LeftClimber m_LeftClimberSubsystem = new LeftClimber();
  private final RightClimber m_RightClimberSubsystem = new RightClimber();

  private final LeftClimberUp m_LeftClimberUp = new LeftClimberUp(m_LeftClimberSubsystem);
  private final LeftClimberDown m_LeftClimberDown = new LeftClimberDown(m_LeftClimberSubsystem);

  private final RightClimberUp m_RightClimberUp = new RightClimberUp(m_RightClimberSubsystem);
  private final RightClimberDown m_RightClimberDown = new RightClimberDown(m_RightClimberSubsystem);

  public RobotContainer() {

    m_chooser.setDefaultOption("Middle", middleRoutine());
    m_chooser.addOption("MiddlePID", middlePIDRoutine());
    m_chooser.addOption("redAlliance_threeNotePID", redAlliance_threeNotePID());
    
    SmartDashboard.putData("Auto choices", m_chooser);

    m_drivetrainSubsystem.setDefaultCommand(m_DriveWithJoystick);
    
    
    //Arm
    // DONT ACTIVATE SETPOINT FROM 0.45 TO 0.62 IF CLIMBERS ARE UP
    L32.whileTrue(m_ArmSubsystem.setSetpoint(0.16)); // Intake / Modo Correr 2

    povDown2.whileTrue(m_ArmSubsystem.setSetpoint(0.2448)); // Shoot
    povUp2.whileTrue(m_ArmSubsystem.setSetpoint(0.4536).unless(() ->  (m_LeftClimberSubsystem.LeftisUp() || m_RightClimberSubsystem.RightisUp()))); // Position 1: 90 degrees
    povLeft2.whileTrue(m_ArmSubsystem.setSetpoint(0.65).unless(() ->  ((m_LeftClimberSubsystem.LeftisUp() || m_RightClimberSubsystem.RightisUp()) && (m_ArmSubsystem.isOnFront())))); // Position 2: 90 degrees
    povRight2.whileTrue(m_ArmSubsystem.setSetpoint(0.50).unless(() ->  ((m_LeftClimberSubsystem.LeftisUp() || m_RightClimberSubsystem.RightisUp()) && (m_ArmSubsystem.isOnFront())))); // Position 3: 90 degrees

/*     xButton2.whileTrue(m_Arm_manualSetpointFront);
    bButton2.whileTrue(m_Arm_manualSetpointBack);  */
 
    //Intake
    aButton2.toggleOnTrue(m_Intake_getNote); //Intake get Note
    yButton2.onTrue(m_Intake_returnNote); //Intake return Note
    
    //Launcher
    LB2.toggleOnTrue(m_LauncherWithJoystick); //Toggle Shoot
    RB2.whileTrue(m_Intake_throwNote); //Intake throw Note 
    
    //Climbers
   /*  start1.whileTrue((m_RightClimberUp).unless(() -> m_ArmSubsystem.isUp()));
    back1.whileTrue((m_LeftClimberUp).unless(() -> m_ArmSubsystem.isUp())); */
    LB1.whileTrue(m_LeftClimberDown);
    RB1.whileTrue(m_RightClimberDown);


    //SysID Triggers
    /* aButton.whileTrue(m_LauncherSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    bButton.whileTrue(m_LauncherSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    xButton.whileTrue(m_LauncherSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    yButton.whileTrue(m_LauncherSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */
 
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command middleRoutine(){
    return new SequentialCommandGroup( //
    // Initial Set and Shoot
 /*    m_ArmSubsystem.autoSetSetpoint(0.12), //
    m_LauncherSubsystem.autoLaunchOn(), //
    Commands.waitSeconds(4.0).asProxy(), // Aqui iria condicional de que si se llego a los RPS
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(1.0).asProxy(), //
    m_LauncherSubsystem.autoLaunchOff(), //s
    m_IntakeSubsystem.autoIntakeShootOff(), //
    m_ArmSubsystem.autoSetSetpoint(0.005), //
    Commands.waitSeconds(2.0).asProxy(), //  */
    
    //Aqui meter condicional de que si llega a setpoint < 0.010 haga el siguiente comando
    // Go to Next Note
          new ParallelCommandGroup( //
            m_IntakeSubsystem.autoGetNote().until(() -> m_IntakeSubsystem.noteIn()), //
            m_drivetrainSubsystem.driveDistanceCommand(18, AutoConstants.kDriveSpeed)
            .withTimeout(AutoConstants.kTimeoutSeconds) //
            .until(() -> m_IntakeSubsystem.noteIn()) //
          )); //

 /*    m_ArmSubsystem.autoSetSetpoint(0.166), //
    Commands.waitSeconds(1.0).asProxy(), //
    m_LauncherSubsystem.autoLaunchOn(), //
    Commands.waitSeconds(5.0).asProxy(), //
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(2.0).asProxy(), //
    m_LauncherSubsystem.autoLaunchOff(), //
    m_IntakeSubsystem.autoIntakeShootOff(), //
    m_ArmSubsystem.autoSetSetpoint(0.005) //   aqui falta una coma
    



    /* m_drivetrainSubsystem.autoTurnLeft(90), //
    
    // Go to 3rd Note
    new ParallelCommandGroup( //
    m_IntakeSubsystem.autoGetNote().until(() -> m_IntakeSubsystem.noteIn()), //
    m_drivetrainSubsystem.driveDistanceCommand(20, AutoConstants.kDriveSpeed)
    .withTimeout(AutoConstants.kTimeoutSeconds).until(() -> m_IntakeSubsystem.noteIn()) //
    ), //

    m_drivetrainSubsystem.autoTurnLeft(45), //
    m_ArmSubsystem.autoSetSetpoint(0.1524), //
    Commands.waitSeconds(1.0).asProxy(), //
    m_LauncherSubsystem.autoLaunchOn(), //
    Commands.waitSeconds(3.0).asProxy(), //
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(1.0).asProxy(), //
    m_LauncherSubsystem.autoLaunchOff(), //
    m_IntakeSubsystem.autoIntakeShootOff(), //
    m_ArmSubsystem.autoSetSetpoint(0.005) //  */  



    //); // */
  }

  public Command middlePIDRoutine(){
    return new SequentialCommandGroup( //
    // Initial Set and Shoot
    /* m_ArmSubsystem.autoSetSetpoint(0.12), //
    m_LauncherSubsystem.autoLaunchOn(), //
    Commands.waitSeconds(3.0).asProxy(), // Aqui iria condicional de que si se llego a los RPS
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(1.0).asProxy(), //
    m_LauncherSubsystem.autoLaunchOff(), //
    m_IntakeSubsystem.autoIntakeShootOff(), //
    m_ArmSubsystem.autoSetSetpoint(0.005), //
    Commands.waitSeconds(2.0).asProxy(), //  */
    
    //Aqui meter condicional de que si llega a setpoint < 0.010 haga el siguiente comando
    // Go to Next Note
          new ParallelCommandGroup( //
        /*     m_IntakeSubsystem.autoGetNote()
            .until(() -> m_IntakeSubsystem.noteIn()), */
            m_drivetrainSubsystem.calculatePID_drive(2, 2, 0.7) //primero va el setpoint derecho y luego el setpoint izquierdo (no poner negativo para ir hacia adelante, el metodo ya lo hace)
            .until(() -> m_IntakeSubsystem.noteIn())
          )

  

    ); 
  }
  
  public Command redAlliance_threeNotePID(){
    return new SequentialCommandGroup( //
    /* 1. Avanzar dos metros con intake prendido y apagar intake cuando se detecte nota
     * 2. Retroceder dos metros mientras ajustamos brazo y prendemos lanzador
     * 3. Esperar 3 segundos a que cargue el lanzador y disparar (se va a reemplazar por setpoint cuando tengamos el encoder)
     * 4, Apagar lanzador, intake y bajar brazo
     * 5. Cuando el setpoint del brazo este abajo. Avanzar dos metros
     * 5. Girar 90 grados a la derecha
     * 6. Avanzar 1.48 metros con el intake prendido y apagar intake cuando se detecte nota
     * 7. Girar 120 grados a la derecha mientras cargamos shooter
     * 8. Acercarnos 1.5 metros mientras levantamos brazo 
     * 9. Disparar 
    */

    // Initial Set and Shoot
    m_ArmSubsystem.autoSetSetpoint(0.12), //
    m_LauncherSubsystem.autoLaunchOn(), //
    Commands.waitSeconds(3.0).asProxy(), // Aqui iria condicional de que si se llego a los RPS
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(1).asProxy(), //
    m_LauncherSubsystem.autoLaunchOff(), //
    m_IntakeSubsystem.autoIntakeShootOff(), //
    m_ArmSubsystem.autoSetSetpoint(0.002), //
    Commands.waitUntil((() -> m_ArmSubsystem.autoRunMode())), //
    
    //Aqui meter condicional de que si llega a setpoint < 0.010 haga el siguiente comando
    // Go to Next Note
    new ParallelCommandGroup( //
      m_drivetrainSubsystem.calculatePID_drive(2, 2, 0.5), //primero va el setpoint derecho y luego el setpoint izquierdo (no poner negativo para ir hacia adelante, el metodo ya lo hace)
      m_IntakeSubsystem.autoGetNote() //
      .until(() -> m_IntakeSubsystem.noteIn()) //
      ), //
      
    new ParallelCommandGroup( //
    m_drivetrainSubsystem.calculatePID_drive(-1.92, -1.92, 0.7), //
    m_ArmSubsystem.autoSetSetpoint(0.12), //
    m_LauncherSubsystem.autoLaunchOn()//
    ),

    Commands.waitSeconds(1.0).asProxy(), // Aqui iria condicional de que si se llego a los RPS
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(1).asProxy(), //
    m_LauncherSubsystem.autoLaunchOff(), //
    m_IntakeSubsystem.autoIntakeShootOff(), //
    m_ArmSubsystem.autoSetSetpoint(0.002) //

    ); 
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
