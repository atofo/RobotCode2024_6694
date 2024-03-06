package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public class LEDConstants {
        public static final int pin0 = 6;
        public static final int pin1 = 5;
        public static final int pin2 = 4;
    }

    public class OperatorConstants {
        public static final int firstcontrollerPort = 0;
        public static final int secondcontrollerPort = 1;
    }

    public class AutoConstants{
    public static final double kTimeoutSeconds = 10; //El tiempo que va estar haciendo eso
    public static final double kDriveDistanceMeters = 20;
    public static final double kDriveSpeed = 0.30;
    }

    public class DrivetrainConstants {
        public static final int leftFrontMotor_PORT = 13;
        public static final int rightFrontMotor_PORT = 12;
        public static final int leftRearMotor_PORT = 14;
        public static final int rightRearMotor_PORT = 11;

        public static final double kP = 0.53;
        public static final double kI = 0.00005;
        public static final double kD = 0.01;


        public static final double kGearRatio = 12.76;
        public static final int kEncoderCPR = 535; // 42 CPRs del NEO * 12.76:1 (transmision de las mecanums), probablmente habra que multiplicar la transmision en otro lado y no aqui
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
        
        //public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kEncoderConversionFactor = (Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * kWheelRadiusMeters) * 10));
        
        public static final double kgyrokP   = 0.0002;
    }

    public class PIDConstants {
        public static final double kP = 4.2;
        public static final double kI = 0.0080;
        public static final double kD = 0.0032;
    }

    public class ArmConstants {
        public static final int arm_leftMotor_PORT = 21;
        public static final int arm_rightMotor_PORT = 22;
        public static final double kEncoderError = 0.2332;
        public static final double kAtSetpointTolerance = 0.020;
        public static final double kAtSetpointBelowSpeakerTolerance = 0.004;
    }

    public class IntakeLauncherConstants {
        public static final int intakelauncher_intakeMotor_PORT = 33;
        public static final int intakelauncher_downMotor_PORT = 32;
        public static final int intakelauncher_upMotor_PORT = 31;
        public static final int intakelauncher_intakeSwitch_PORT = 3;
    }

    public class ClimberConstants {
        public static final int climber_leftMotor_PORT = 42;
        public static final int climber_rightMotor_PORT = 41;
    }

    public class ShooterConstants {
        /* El Motor NEO puede maximo ~5800 RPM. Si necesitamos RPS (Rotaciones por segundo) hay que dividir entre 60
         5800 / 60 = 96.66 */
        public static final int[] kEncoderPorts = new int[] {1, 2};
        public static final boolean kEncoderReversed = true;
        public static final int kEncoderCPR = 2048; // REV Through Bore Encoder CPR
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations
            1.0 / (double) kEncoderCPR;
    
        public static final double kShooterFreeRPS = 93; //El maximo de RPS que puede dar
        public static final double kShooterTargetRPS = 90; //Al que queremos llegar
        public static final double kShooterToleranceRPS = 3; //Tolerancia de error 
    
        // Ojo, valores experimentales de kP, kI y kD, hay que experimentar.
        //public static final double kP = 0.00012;
        public static final double kP = 0.0000026; // Estaba en 0.0000025
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0;
        public static final double kUpFF = 0.000216; // PR 0.000400
        public static final double kDownFF = 0.000200; // PR 0.000400
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        public static final double maxRPM_up = 4880;
        public static final double maxRPM_down = 4880;
        public static final double RPMmargin = 130;
    
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVolts = 0.19; // Valor ajustado al lanzador
        public static final double kVVoltSecondsPerRotation =
            // Should have value 12V at free speed...
            12.0 / kShooterFreeRPS;
            } 

}
