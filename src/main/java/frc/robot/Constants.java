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
        public static class Drive {
            public static class Rotation {
                public static double KP = 8;
                public static double KI = 0;
            }

            public static class Position {
                public static double KP = 1;
            }
        }

        public static class Elevator {
            public static class Sensors {
                public final static int HALL_EFFECT_PORT = 0;
            }

            public static class Motors {

                public final static int MASTER_CAN_ID = 30;
                public final static int RIGHT_SLAVE_CAN_ID = 31;
                public final static int LEFT_SLAVE_1_CAN_ID = 32;
                public final static int LEFT_SLAVE_2_CAN_ID = 33;
            }

            public static class Physical {
                public final static double WHEEL_RADIUS_IN_METERS = 0.0213995;
                public final static double GEAR_RATIO = 8;
                public final static double MAX_ACCELERATION_IN_MPS_SQUARED = 10;
                public final static double MAX_HEIGHT_IN_METERS = 0.185;
                public final static double POSITION_CONVERSION_FACTOR = WHEEL_RADIUS_IN_METERS * 2 * Math.PI
                        / GEAR_RATIO;
                public final static double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
                public final static double MAX_VELOCITY_IN_MPS = 5676 * VELOCITY_CONVERSION_FACTOR;
            }

            public static class Controls {

                public static final double HEIGHT_THRESHOLD_IN_METERS = 0.005;
                public static final double ELEVATOR_PERCENTAGE = 0.5;
            }

            public static class PID {

                public final static double KP = 5;
                public final static double KI = 0;
                public final static double KD = 0;
            }
        }

        public static class Intake {

            public static class Motors {

                public final static int UPPER_MASTER_SPARK_FLEX_ID = 41;
                public final static int LOWER_SLAVE_SPARK_FLEX_ID = 40;

            }

            public static class Physical {

                public final static double INTAKE_PERCENTAGE = 0.5;
                public static final double DECELERATION_THRESHOLD = -80; // how many rpm declerates in one iteration
                public static final double INTAKE_VELOCIRTY_THREASHOLD = 500;
            }

            public static class PID {
                public static final double KP = 5;
                public static final double KD = 0;
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

    public static class IO {
        public static final int DRIVER_XBOX_PORT = 0;
        public static final int MECHANISM_XBOX_PORT = 1;
    }

    public static class States {

        public static final class Climb {
            public static double ELEVATOR_HEIGHT = 0.2;
            public static double ELEVATOR_DOWN = 0;
        }

        public static class Intake {
            public static double RADIUS_IN_METERS = 999;
            public static double ELEVATOR_HEIGHT = 0.3;
            public static double VELOCITY_THREASHOLD = 500;
            public static double INTAKE_THREASHOLD = 40;
            public static double INTAKE_PERCEHNTAGE = 0.3;

        }

        public static class Score {
            public static double RADIUS_IN_METERS = 1;

            public static double ELEVATOR_HEIGHT = 0;
            public static double INTAKE_PERCNETAGE = -1;
            public static double INTAKE_TIME = 1;
        }

        public static class None {
            public static double ELEVATOR_HEIGHT = 0;
        }
    }
}
