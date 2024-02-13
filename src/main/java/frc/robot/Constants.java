package frc.robot;

public class Constants {

    public class OperatorConstants {
        public static final int controllerPort = 0;
    }

    public class DrivetrainConstants {
        public static final int leftFrontMotor_PORT = 11;
        public static final int rightFrontMotor_PORT = 22;
        public static final int leftRearMotor_PORT = 25;
        public static final int rightRearMotor_PORT = 12;
    }

    public class PIDConstants {

        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0.05;

    }

    public class ArmConstants {
        public static final int arm_leftMotor_PORT = 14;
        public static final int arm_rightMotor_PORT = 31;
    }

    public class IntakeLauncherConstants {
        public static final int intakelauncher_intakeMotor_PORT = 30;
        public static final int intakelauncher_downMotor_PORT = 33;
        public static final int intakelauncher_upMotor_PORT = 32;
        public static final int intakelauncher_intakeSwitch_PORT = 3;
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
