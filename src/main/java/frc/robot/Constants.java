// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Subsystems {

        public static class Elevator {

            public static class Motors {

                public final static int MASTER_CAN_ID = 0;
                public final static int SLAVE_1_CAN_ID = 0;
                public final static int SLAVE_2_CAN_ID = 0;
                public final static int SLAVE_3_CAN_ID = 0;
            }

            public static class Physical {

                public final static double WHEEL_RADIUS_IN_METERS = 0.0;
                public final static double GEAR_RATIO = 0.0;
                public final static double MAX_VELOCITY_IN_MPS = 0;
                public final static double MAX_ACCELERATION_IN_MPS_SQUARED = 0;
                public final static double MAX_HEIGHT_IN_METERS = 0;

            }

            public static class Controls {
                public static final double HEIGHT_THRESHOLD_IN_METERS = 0;
                public static final double ELEVATOR_PERCENTAGE = 0;
            }

            public static class PID {

                public final static double KP = 0;
                public final static double KI = 0;
                public final static double KD = 0;
            }
        }

        public static class Intake {
            public static class Motors {
                public final static int UPPER_MASTER_SPARK_FLEX_ID = 1;
                public final static int LOWER_SLAVE_SPARK_FLEX_ID = 1;

            }

            public static class Physical {
                public final static double INTAKE_PERCENTAGE = 0.3;
            }
        }

        public static class DeepCage {
            public static class Motors {
                public final static int DEEP_CAGE_MAX_MOTOR_ID = 5;
            }

            public static class Sensors {
                public final static int LEG_LIMIT_SWITCH_CHANNEL = 0;
            }

            public static class Physical {
                public final static double percentage = 0.3;
            }
        }
    }

    public static class OperatorConstants {

        public static final int kDriverControllerPort = 0;
    }
}
