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

        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0.05;

    }

    public class ArmConstants {
        public static final int arm_leftMotor_PORT = 21;
        public static final int arm_rightMotor_PORT = 22;
    }

    public class IntakeLauncherConstants {
        public static final int intakelauncher_intakeMotor_PORT = 33;
        public static final int intakelauncher_downMotor_PORT = 31;
        public static final int intakelauncher_upMotor_PORT = 32;
        public static final int intakelauncher_intakeSwitch_PORT = 2;
    }

    public class ShooterFeedforward {
        public static final double launcher_kS = 0.19;


    }

}
