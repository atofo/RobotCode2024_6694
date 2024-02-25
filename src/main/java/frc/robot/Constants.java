package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

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
        


    }

    public class PIDConstants {

        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0.05;

    }

    public class ArmConstants {
        public static final int arm_leftMotor_PORT = 21;
        public static final int arm_rightMotor_PORT = 22;
        public static final double kEncoderError = 0.6275;
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

    public class PIDConstants_Launcher {
        /* public static final double kP = 0.0008; //Constantes PID + Ff viejo
        public static final double kI = 0;
        public static final double kD = 0; */

        public static final double kS = 0.19;
        public static final double kV = 0.19;
        public static final double kA = 0.48; 
        
        public static final double kP = 0.0002;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kFF_downMotor = 0.00017;
        public static final double kFF_upMotor = 0.00017;

        public static final double minPIDOutput = 1.0;
        public static final double maxPIDOutput = 0.0;
        }

}
