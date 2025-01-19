// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class DriveConstants {
		public static class Dimensions {
			/**
             * Distance between the center of the right wheels to the center of the left
             * wheels (Meters)
             */
            public static final double kTrackWidthMeters = 85.5;

            /**
             * +
             * Distance between the center of the back wheels to the center of the front
             * wheels (Meters)
             */
            public static final double kWheelbaseMeters = 85.5;

            /**
             * The ratio between the Motor and the center wheel of the Swerve module (which
             * the CANcoder lies on)
             */
            public static final double kRotorToSensorRatioDrive = 8.14;
            public static final double kRotorToSensorRatioSteer = 150 / 7;

		}
		public static class ID {
			// ! ------ DO NOT USE THESE VALUES IN PRODUCTION :( ------
			public static final int kFrontLeftDriveMotor = 0;
			public static final int kFrontLeftSteerMotor = 1;
			public static final int kFrontLeftEncoder = 0;

			public static final int kFrontRightDriveMotor = 2;
			public static final int kFrontRightSteerMotor = 3;
			public static final int kFrontRightEncoder = 1;

			public static final int kBackLeftDriveMotor = 4;
			public static final int kBackLeftSteerMotor = 5;
			public static final int kBackLeftEncoder = 2;

			public static final int kBackRightDriveMotor = 6;
			public static final int kBackRightSteerMotor = 7;
			public static final int kBackRightEncoder = 3;


		}
		public static class PID {
			public static class Drive {
				/**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double kS = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double kV = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what
                 * number to put)
                 */
                public static final double kA = 0.0;

                /**
                 * Proportional tuning - error
                 * Lower the kP
                 */
                public static final double kP = 0.00035;
                /**
                 * 
                 * Integral tuning - learning
                 */
                public static final double kI = 0.000002;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double kD = 0.0;
			}
			public static class Steer {
				/**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double kS = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double kV = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what
                 * number to put)
                 */
                public static final double kA = 0.0;

                /**
                 * Proportional tuning - error
                 */
                public static final double kP = 10.0;
                /**
                 * Integral tuning - learning
                 */
                public static final double kI = 1.0;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double kD = 0.0;
			}
		}

	}
}
