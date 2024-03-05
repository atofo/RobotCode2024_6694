// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Arm_manualSetpointFront;
import frc.robot.Commands.DriveInverted;
import frc.robot.Commands.Arm_manualSetpointBack;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.DriveWithJoystick;
import frc.robot.Commands.Intake_Emergency;
import frc.robot.Commands.Intake_getNote;
import frc.robot.Commands.Intake_returnNote;
import frc.robot.Commands.LeftClimberDown;
import frc.robot.Commands.LeftClimberUp;
import frc.robot.Commands.RightClimberUp;
import frc.robot.Commands.Shooter_Emergency;
import frc.robot.Commands.RightClimberDown;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.LeftClimber;
import frc.robot.Subsystems.RightClimber;
import edu.wpi.first.wpilibj.DigitalOutput;

public class RobotContainer {
  // Sendable Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  private final CommandXboxController m_firstDriverController = new CommandXboxController(
      OperatorConstants.firstcontrollerPort);
  private final CommandXboxController m_secondDriverController = new CommandXboxController(
      OperatorConstants.secondcontrollerPort);

  // First Driver Triggers
  private Trigger L31 = m_firstDriverController.leftStick();
  private Trigger aButton1 = m_firstDriverController.a();
  private Trigger bButton1 = m_firstDriverController.b();
  private Trigger xButton1 = m_firstDriverController.x();

  private Trigger start1 = m_firstDriverController.start();
  private Trigger back1 = m_firstDriverController.back();

  private Trigger LB1 = m_firstDriverController.leftBumper();
  private Trigger RB1 = m_firstDriverController.rightBumper();

  // Second Driver Triggers
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

  private Trigger start2 = m_secondDriverController.start();
  private Trigger back2 = m_secondDriverController.back();

  private Trigger L32 = m_secondDriverController.leftStick();
  private Trigger R32 = m_secondDriverController.rightStick();

  // Drivetrain
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final DriveWithJoystick m_DriveWithJoystick = new DriveWithJoystick(m_drivetrainSubsystem,
      () -> -m_firstDriverController.getRawAxis(0),
      () -> m_firstDriverController.getRawAxis(1),
      () -> -m_firstDriverController.getRawAxis(4),
      () -> m_firstDriverController.getRawAxis(3),
      () -> m_firstDriverController.getRawAxis(2));

  private final DriveInverted m_driveInverted = new DriveInverted(m_drivetrainSubsystem,
      () -> -m_firstDriverController.getRawAxis(0),
      () -> m_firstDriverController.getRawAxis(1),
      () -> m_firstDriverController.getRawAxis(4),
      () -> m_firstDriverController.getRawAxis(3),
      () -> m_firstDriverController.getRawAxis(2));

  // Arm
  public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final Arm_manualSetpointFront m_Arm_manualSetpointFront = new Arm_manualSetpointFront(m_ArmSubsystem);
  private final Arm_manualSetpointBack m_Arm_manualSetpointBack = new Arm_manualSetpointBack(m_ArmSubsystem);

  // Intake
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final Intake_getNote m_Intake_getNote = new Intake_getNote(m_IntakeSubsystem);
  private final Intake_returnNote m_Intake_returnNote = new Intake_returnNote(m_IntakeSubsystem);
  private final Intake_Emergency m_Intake_Emergency = new Intake_Emergency(m_IntakeSubsystem);

  // Shooter
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final Command m_stopShooter = Commands.runOnce(m_shooter::disable, m_shooter);
  private final Command m_stopShooter2 = Commands.runOnce(m_shooter::disable, m_shooter);

  private final Command m_spinUpShooter = Commands.run(m_shooter::enable, m_shooter)
      .until(() -> !m_IntakeSubsystem.noteIn()).andThen((new SequentialCommandGroup(
          Commands.waitSeconds(1.5).asProxy(),
          m_stopShooter2)));

  private final Shooter_Emergency m_Shooter_Emergency = new Shooter_Emergency(m_shooter);
    
  // Climbers
  private final LeftClimber m_LeftClimberSubsystem = new LeftClimber();
  private final RightClimber m_RightClimberSubsystem = new RightClimber();

  private final LeftClimberUp m_LeftClimberUp = new LeftClimberUp(m_LeftClimberSubsystem);
  private final LeftClimberDown m_LeftClimberDown = new LeftClimberDown(m_LeftClimberSubsystem);

  private final RightClimberUp m_RightClimberUp = new RightClimberUp(m_RightClimberSubsystem);
  private final RightClimberDown m_RightClimberDown = new RightClimberDown(m_RightClimberSubsystem);

  // LEDS

  public DigitalOutput port0 = new DigitalOutput(LEDConstants.pin0);
  public DigitalOutput port1 = new DigitalOutput(LEDConstants.pin1);
  public DigitalOutput port2 = new DigitalOutput(LEDConstants.pin2);

  public RobotContainer() {
    m_chooser.setDefaultOption("redAlliance_threeNotePID", redAlliance_threeNotePID());
    m_chooser.addOption("redAlliance_4Note", redAlliance_4Note());
    m_chooser.addOption("blueAlliance_threeNotePID", blueAlliance_threeNotePID());
    m_chooser.addOption("fwd test", fwdTest());
    SmartDashboard.putData("Auto choices", m_chooser);

    // LEDS

    // Drivetrain
    m_drivetrainSubsystem.setDefaultCommand(m_DriveWithJoystick);

    //bButton1.onTrue(m_drivetrainSubsystem.StraightApril());
    aButton1.toggleOnTrue(m_driveInverted);

    // Climbers
/*  LB1.whileTrue(m_LeftClimberUp); // Left Climber Up
    start1.whileTrue(m_RightClimberDown); // Right Climber Down
    RB1.whileTrue(m_RightClimberUp); // Right Climber Up
    back1.whileTrue(m_LeftClimberDown); // Left Climber Down */

    // Arm
    // DONT ACTIVATE SETPOINT FROM 0. TO 0. IF CLIMBERS ARE UP
    L32.whileTrue(m_ArmSubsystem.setAmplifierSetpoint(0.001)); // Intake / Modo Correr 2
    povRight2.whileTrue(m_ArmSubsystem.setSetpoint(0.314)); // Climb 1
    povLeft2.whileTrue(m_ArmSubsystem.setAmplifierSetpoint(0.2615)); // Climb 2 // ARRIBA
    // back2.whileTrue(m_ArmSubsystem.setSetpoint(0.122)); // Abajo de Speaker
    back2.whileTrue(m_ArmSubsystem.setAmplifierSetpoint(0.100)); // Abajo de Speaker
    start2.whileTrue(m_ArmSubsystem.setAmplifierSetpoint(0.150)); // Brazo detras de linea
    RT2.whileTrue(m_Arm_manualSetpointFront); //Manual Enfrente
    LT2.whileTrue(m_Arm_manualSetpointBack); //Manual Atras
    R32.whileTrue(m_ArmSubsystem.setSetpoint(0.314)); // AMP
    bButton2.onTrue(m_ArmSubsystem.setAprilSetpoint(() -> m_drivetrainSubsystem.limelightArea())); // Brazo Limelight

    // Intake
    aButton2.toggleOnTrue(m_Intake_getNote); // Intake get Note
    yButton2.whileTrue(m_Intake_returnNote);
    povDown2.whileTrue(m_Intake_Emergency);

    // Shooter
    RB2.onTrue(m_spinUpShooter); // Empezar a girar lanzador
    LB2.onTrue(m_stopShooter); // Parar lanzador
    //povUp2.whileTrue(m_Shooter_Emergency); // Emergency Shoot
    povUp2.toggleOnTrue(m_Shooter_Emergency); // Emergency Shoot


    
    // SysID Triggers
    /*
     * aButton.whileTrue(m_LauncherSubsystem.sysIdQuasistatic(SysIdRoutine.Direction
     * .kForward));
     * bButton.whileTrue(m_LauncherSubsystem.sysIdQuasistatic(SysIdRoutine.Direction
     * .kReverse));
     * xButton.whileTrue(m_LauncherSubsystem.sysIdDynamic(SysIdRoutine.Direction.
     * kForward));
     * yButton.whileTrue(m_LauncherSubsystem.sysIdDynamic(SysIdRoutine.Direction.
     * kReverse));
     */

    configureBindings();
  }

 
  private void configureBindings() {
    Command shoot = Commands.either(
        // Run the feeder
        Commands.runOnce(m_IntakeSubsystem::throwNote, m_IntakeSubsystem), // Va a lanzar si se alcanza la velocidad
                                                                           // deseada
        // Do nothing
        Commands.none(),
        // Determine which of the above to do based on whether the shooter has reached
        // the
        // desired speed
        () -> (m_shooter.atSetpoint() && m_ArmSubsystem.atSetpoint()));

    Command stopIntake = Commands.runOnce(m_IntakeSubsystem::intakeOFF, m_IntakeSubsystem);

    // disparar cuando se presione el boton X
    xButton2.onTrue(shoot).onFalse(stopIntake);
  }


  public Command fwdTest(){
    return m_drivetrainSubsystem.calculatePID_drive(1.8, 1.8, 0.32, 1.7);
  }



  // ROUTINES

  public Command redAlliance_threeNotePID() {
    return new SequentialCommandGroup( //

    // NOTE 0
      new ParallelCommandGroup(
      m_ArmSubsystem.autoSetAmplifierSetpoint(0.106), // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
      m_shooter.autoEnable(), //
      //Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(4.5)
      Commands.waitUntil(() -> (m_shooter.atSetpoint()) && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(1.7)
      ), //
    
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(.6).asProxy(), //

      new ParallelCommandGroup(
      m_shooter.autoDisable(), //
      m_IntakeSubsystem.autoIntakeShootOff(), //
      m_ArmSubsystem.autoSetAmplifierSetpoint(0.001) //
      ),


    // FIRST NOTE PICK AND SHOOT
       new ParallelCommandGroup(
        m_drivetrainSubsystem.calculatePID_drive(1.8, 1.8, 0.32, 1.7),
        m_IntakeSubsystem.autoGetNote() //
        .until(() -> m_IntakeSubsystem.noteIn()) //
        ),

        
        new ParallelCommandGroup(
          m_drivetrainSubsystem.calculatePID_drive(-1.5, -1.5, 0.645, 1.7),
          m_ArmSubsystem.autoSetAmplifierSetpoint(0.1275), // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
          m_shooter.autoEnable(),
          //Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpoint()).withTimeout(4)
          Commands.waitUntil(() -> (m_shooter.atSetpoint()) && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(1.2)
        ),

  m_IntakeSubsystem.autoIntakeShootOn(), //
  Commands.waitSeconds(.6).asProxy(), //
  m_shooter.autoDisable(), //
  m_IntakeSubsystem.autoIntakeShootOff(), //
  m_ArmSubsystem.autoSetSetpoint(0.001), //

  // SECOND NOTE PICK AND SHOOT
         new ParallelCommandGroup(
        m_drivetrainSubsystem.calculatePID_mecanumdrive(-0.73, 4.35, 0.6, 1000) //
        .until(() -> m_IntakeSubsystem.noteIn()), //
        m_IntakeSubsystem.autoGetNote() //
        .until(() -> m_IntakeSubsystem.noteIn()) //
        ),

        new ParallelCommandGroup(
          m_shooter.autoEnable(), //
          m_drivetrainSubsystem.calculatePID_drive(-.4, .4, 1, 3), //
          m_ArmSubsystem.autoSetAmplifierSetpoint(0.147), // // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
          Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpoint()).withTimeout(2) //
        ),


        m_IntakeSubsystem.autoIntakeShootOn(), //
        Commands.waitSeconds(.6).asProxy(), //

        new ParallelCommandGroup(
          m_IntakeSubsystem.autoIntakeShootOff(), //
          m_shooter.autoDisable(), //
          m_ArmSubsystem.autoSetAmplifierSetpoint(0.001) //
        )
    );
  }




  public Command blueAlliance_threeNotePID() {
    return new SequentialCommandGroup( //

    // NOTE 0
      new ParallelCommandGroup(
      m_ArmSubsystem.autoSetAmplifierSetpoint(0.106), // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
      m_shooter.autoEnable(), //
      //Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(4.5)
      Commands.waitUntil(() -> (m_shooter.atSetpoint()) && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(1.7)
      ), //
    
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(.6).asProxy(), //

      new ParallelCommandGroup(
      m_shooter.autoDisable(), //
      m_IntakeSubsystem.autoIntakeShootOff(), //
      m_ArmSubsystem.autoSetAmplifierSetpoint(0.001) //
      ),


    // FIRST NOTE PICK AND SHOOT
       new ParallelCommandGroup(
        m_drivetrainSubsystem.calculatePID_drive(1.8, 1.8, 0.32, 100)
        .until(() -> m_IntakeSubsystem.noteIn()), //
        m_IntakeSubsystem.autoGetNote() //
        .until(() -> m_IntakeSubsystem.noteIn()) //
        ),

        
        new ParallelCommandGroup(
          m_drivetrainSubsystem.calculatePID_drive(-1.5, -1.5, 0.645, 1.7),
          m_ArmSubsystem.autoSetAmplifierSetpoint(0.1275), // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
          m_shooter.autoEnable(),
          //Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpoint()).withTimeout(4)
          Commands.waitUntil(() -> (m_shooter.atSetpoint()) && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(1.2)
        ),

  m_IntakeSubsystem.autoIntakeShootOn(), //
  Commands.waitSeconds(.6).asProxy(), //
  m_shooter.autoDisable(), //
  m_IntakeSubsystem.autoIntakeShootOff(), //
  m_ArmSubsystem.autoSetSetpoint(0.001), //

  // SECOND NOTE PICK AND SHOOT
         new ParallelCommandGroup(
        m_drivetrainSubsystem.calculatePID_mecanumdrive(3.8, -0.75, 0.6, 1000) // AJUSTAR EL ROCKET PLANNER
        .until(() -> m_IntakeSubsystem.noteIn()), //
        m_IntakeSubsystem.autoGetNote() //
        .until(() -> m_IntakeSubsystem.noteIn()) //
        ),

        new ParallelCommandGroup(
          m_shooter.autoEnable(), //
          m_drivetrainSubsystem.calculatePID_drive(.4, -.4, 1, 3), //
          m_ArmSubsystem.autoSetAmplifierSetpoint(0.147), // // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
          Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpoint()).withTimeout(2) //
        ),


        m_IntakeSubsystem.autoIntakeShootOn(), //
        Commands.waitSeconds(.6).asProxy(), //

        new ParallelCommandGroup(
          m_IntakeSubsystem.autoIntakeShootOff(), //
          m_shooter.autoDisable(), //
          m_ArmSubsystem.autoSetAmplifierSetpoint(0.001) //
        )
    );
  }




  public Command redAlliance_4Note(){
    return new SequentialCommandGroup(

    // NOTE 0
    new ParallelCommandGroup(
      m_ArmSubsystem.fourNote_autoSetAmplifierSetpoint(0.1015), // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
      m_shooter.autoEnable(), //
      //Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(4.5)
      Commands.waitUntil(() -> (m_shooter.atSetpoint()) && m_ArmSubsystem.atSetpointBelowSpeaker()).withTimeout(1.4)
      ), //
    
    m_IntakeSubsystem.autoIntakeShootOn(), //
    Commands.waitSeconds(.5).asProxy(), //
    
    new ParallelCommandGroup(
      m_shooter.autoDisable(), //
      m_IntakeSubsystem.autoIntakeShootOff(), //
      m_ArmSubsystem.autoSetAmplifierSetpoint(0.001) //
      ),
      
    //FIRST NOTE
    new ParallelCommandGroup(
      m_drivetrainSubsystem.calculatePID_drive(1.8, 1.8, 0.35, 100)
      .until(() -> m_IntakeSubsystem.noteIn()), //
      m_IntakeSubsystem.autoGetNote() //
      .until(() -> m_IntakeSubsystem.noteIn()) //
      ),

      new ParallelCommandGroup(
        m_drivetrainSubsystem.calculatePID_drive(-0.63, -0.63, 0.4, 2.2),
        m_shooter.autoEnable(), //
        m_ArmSubsystem.fourNote_autoSetAmplifierSetpoint(0.151), // // AQUI SE CAMBIA EL ANGULO DEL BRAZO, NO SUBIR!!
        Commands.waitUntil(() -> m_shooter.atSetpoint() && m_ArmSubsystem.atSetpoint()).withTimeout(2.2) //
      ),

      m_IntakeSubsystem.autoIntakeShootOn(), //
      Commands.waitSeconds(.5).asProxy(), //

      new ParallelCommandGroup(
        m_IntakeSubsystem.autoIntakeShootOff(), //
        m_shooter.autoDisable(), //
        m_ArmSubsystem.autoSetAmplifierSetpoint(0.001) //
    )
    
    );
  }
 
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
