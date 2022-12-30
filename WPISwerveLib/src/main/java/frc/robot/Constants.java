package frc.robot;

public final class Constants {
    public static final class CAN_ID {
        public static final int DRIVETRAIN_PIGEON_ID = 5;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 19;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 14;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 20;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 15;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 18;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22;
    }

    public static final class SWERVE {
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(162.7); // Was 162.7
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(135.08);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(125);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(9.45);

        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = .60325;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = .62865;

    }
}
