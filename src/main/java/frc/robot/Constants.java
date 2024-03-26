package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.SwerveModuleConstants;

public class Constants {
    
    public static final class Drivebase {
        public static final double WHEEL_LOCK_TIME = 10;

        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final double HEADING_TOLERANCE = Math.toRadians(2);

        public static final boolean CANCODER_INVERT = false;
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean INVERT_GYRO = false;

        public static final double FRONT_LEFT_X = Units.inchesToMeters(11.75);
        public static final double FRONT_LEFT_Y = Units.inchesToMeters(11.75);
        public static final double FRONT_RIGHT_X = Units.inchesToMeters(11.75);
        public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-11.75);
        public static final double BACK_LEFT_X = Units.inchesToMeters(-11.75);
        public static final double BACK_LEFT_Y = Units.inchesToMeters(11.75);
        public static final double BACK_RIGHT_X = Units.inchesToMeters(-11.75);
        public static final double BACK_RIGHT_Y = Units.inchesToMeters(-11.75);

        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
            new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
            new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
            new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
        };

        public static final double MAX_SPEED = Units.feetToMeters(14.5);
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y);

        public static final double FULL_ROT = 360;

        public static final double HEADING_KP = MAX_ANGULAR_VELOCITY / Math.PI;
        // Theoretical max acceleration should be as follows:
        // (NEO stall torque * module gearing * number of modules) / (wheel radius * robot mass) = m/s/s
        // (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS)
        // However, the drive is traction-limited, so the max accelration is actually (wheel coefficient of friction * gravity)
        public static final double MAX_ACCELERATION = 1.19 * 9.81; // COF (blue nitrile on carpet) as reported by Studica

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

         public static final double MODULE_KP = 0.01;
         public static final double MODULE_KI = 0;
         public static final double MODULE_KD = 0;
         public static final double MODULE_IZ = 0;
         public static final double MODULE_KF = 0;
 
         public static final double VELOCITY_KP = 0; 
         public static final double VELOCITY_KI = 0; 
         public static final double VELOCITY_KD = 0;
         public static final double VELOCITY_IZ = 0;
         public static final double VELOCITY_KF = 0;
 
         // Drive feedforward gains
         public static final double KS = 0;
         public static final double KV = 12 / MAX_SPEED; // Volt-seconds per meter (max voltage divided by max speed)
         public static final double KA = 12 / MAX_ACCELERATION; // Volt-seconds^2 per meter (max voltage divided by max accel)

         public static final double METERS_PER_MOTOR_ROTATION = (Math.PI * Units.inchesToMeters(4)) / 6.75;
         public static final double DEGREES_PER_STEERING_ROTATION = 360 / 12.8;

         public static final int NUM_MODULES = 4;

         public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 13;
            public static final double ANGLE_OFFSET = -85;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
         }
         public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 21;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CANCODER_ID = 23;
            public static final double ANGLE_OFFSET = -65;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
         }
         public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 41;
            public static final int ANGLE_MOTOR_ID = 42;
            public static final int CANCODER_ID = 43;
            public static final double ANGLE_OFFSET = 240;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
         }
         public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 31;
            public static final int ANGLE_MOTOR_ID = 32;
            public static final int CANCODER_ID = 33;
            public static final double ANGLE_OFFSET = 203;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
         }
    }
}