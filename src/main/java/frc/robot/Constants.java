package frc.robot;

public class Constants {

    public class OperatorConstants {
        public static final int controllerPort = 0;
    }

    public class DrivetrainConstants {
        public static final int leftFrontMotor_PORT = 14;
        public static final int rightFrontMotor_PORT = 12;
        public static final int leftRearMotor_PORT = 13;
        public static final int rightRearMotor_PORT = 11;
    }

    public class PIDConstants {

        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0.005;

    }

    public class ArmConstants {
        public static final int arm_leftMotor_PORT = 21;
        public static final int arm_rightMotor_PORT = 22;
    }

    public class IntakeLauncherConstants {
        public static final int intakelauncher_intakeMotor_PORT = 33;
        public static final int intakelauncher_downMotor_PORT = 31;
        public static final int intakelauncher_upMotor_PORT = 32;
        public static final int intakelauncher_intakeSwitch_PORT = 3;
    }

    public class ClimberConstants {
        public static final int climber_leftMotor_PORT = 42;
        public static final int climber_rightMotor_PORT = 41;
    }

    public class ShooterConstants {
        /* El Motor NEO puede maximo ~5800 RPM. Si necesitamos RPS (Rotaciones por segundo) hay que dividir entre 60
         5800 / 60 = 96.66 */
        public static final int[] kEncoderPorts = new int[] {0, 1};
        public static final boolean kEncoderReversed = false;
        public static final int kEncoderCPR = 2048; // REV Through Bore Encoder CPR
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations
            1.0 / (double) kEncoderCPR;
    
        public static final double kShooterFreeRPS = 96; //El maximo de RPS que puede dar
        public static final double kShooterTargetRPS = 85; //Al que queremos llegar
        public static final double kShooterToleranceRPS = 0; //Tolerancia de error 
    
        // Ojo, valores experimentales de kP, kI y kD, hay que experimentar.
        public static final double kP = 0.08;
        public static final double kI = 0;
        public static final double kD = 0.002;
    
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVolts = 0.19; // Valor ajustado al lanzador
        public static final double kVVoltSecondsPerRotation =
            // Should have value 12V at free speed...
            12.0 / kShooterFreeRPS;
    
            }   



}
