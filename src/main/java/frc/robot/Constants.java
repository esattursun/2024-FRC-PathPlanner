// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {



  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // Charge Station Constants
    public static final double CHARGE_TIPPING_ANGLE= Math.toRadians(12);
    public static final double CHARGE_TOLERANCE = Math.toRadians(2.5);
    public static final double CHARGE_MAX_SPEED = 0.8;
    public static final double CHARGE_REDUCED_SPEED = 0.70;

    // Delay between reading the gyro and using the value used to aproximate exact angle while spinning (0.02 is one loop)
    public static final double GYRO_READ_DELAY = 0.02;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 2; //7
    public static final int REAR_LEFT_DRIVING_CAN_ID = 8;  //4
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 4; //10
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 6;  //8

    public static final int FRONT_LEFT_TURNING_CAN_ID = 3; //2
    public static final int REAR_LEFT_TURNING_CAN_ID = 9;  //1
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 5;  //3
    public static final int REAR_RIGHT_TURNING_CAN_ID = 7; //6

    public static final boolean GYRO_REVERSED = false;
    public static final Rotation3d GYRO_ROTATION = new Rotation3d(0, 0, - Math.PI / 2);

    public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.05, 0.05, 0.01);
  }


  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS
            / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
            / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    public static final double TURNING_P = 2;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    public static final CANSparkMax.IdleMode DRIVING_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode TURNING_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 20; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 15; // amps
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }

  public static class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);
  }


  public static final class LimelightConstants {

    public static final double goalHeightInches = 53.1496063;//53.1496063
  
  }  

 public static class VisionConstants {
    

    public static final Transform3d PHOTONVISION_TRANSFORM = new Transform3d(
            new Translation3d(0.205697, -0.244475, 0.267365),
            new Rotation3d(0, Units.degreesToRadians(15), 0)
    );

    public static final Vector<N3> PHOTONVISION_STD_DEV = VecBuilder.fill(0.7, 0.7, 0.5);

    public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final double AMBIGUITY_FILTER = 0.05;
  }
}
