package frc.robot;

public class Constants {

    public class OperatorConstants{
        public static final int controllerPort = 0;
    }
    public class DrivetrainConstants {
        public static final int leftFrontMotor_PORT = 14;
        public static final int rightFrontMotor_PORT = 11;
        public static final int leftRearMotor_PORT = 13;
        public static final int rightRearMotor_PORT = 12;
    }

    public class PIDConstants{

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0.001;

    }

    public class ArmConstants {
            public static final int arm_leftMotor_PORT = 14;
            public static final int arm_rightMotor_PORT = 31;
        }

    
}
