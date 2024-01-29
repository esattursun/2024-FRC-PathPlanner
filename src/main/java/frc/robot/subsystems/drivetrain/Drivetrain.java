// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.RobotContainer;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;



public class Drivetrain extends SubsystemBase {

    // Create MAXSwerveModules
    private final SwerveModules swerveModules;

    // The gyro sensor
    private final Gyro gyro;
    public  SwerveModule[] m_SwerveMods;
    /**
     * Creates a new DriveSubsystem.
     */
    public Drivetrain() {

       
    
        if (RobotBase.isSimulation()) {
            this.swerveModules = new SwerveModules(
                    new SIMSwerveModule(DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET),
                    new SIMSwerveModule(DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET),
                    new SIMSwerveModule(DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET),
                    new SIMSwerveModule(DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET)
            );
        } else {
            this.swerveModules = new SwerveModules(
                    new MAXSwerveModule(
                            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
                    ),
                    new MAXSwerveModule(
                            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
                    ),
                    new MAXSwerveModule(
                            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                            DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET
                    ),
                    new MAXSwerveModule(
                            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                            DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET
                    )
            );
        }

        gyro = (RobotBase.isReal() ? new NavXGyro() : new SIMGyro(swerveModules));

        //RobotContainer.swerveTab.addNumber("Front Left", swerveModules.frontLeft::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        //RobotContainer.swerveTab.addNumber("Front Right", swerveModules.frontRight::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        //RobotContainer.swerveTab.addNumber("Back Left", swerveModules.rearLeft::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        //RobotContainer.swerveTab.addNumber("Back Right", swerveModules.rearRight::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);

        //RobotContainer.swerveTab.addNumber("Gyro", () -> gyro.getAngle().getRadians()).withWidget(BuiltInWidgets.kGraph);
    }

    @Override
    public void periodic() {
        RobotContainer.poseEstimation.updateOdometry(
                getRotation(),
                getModulePositions()
        );

        gyro.update();
        swerveModules.update();

        logModuleStates("Swerve State", swerveModules.getStates().asArray());
        logModuleStates("Swerve Set State", new SwerveModuleState[]{
                swerveModules.frontLeft.getSetState(),
                swerveModules.frontRight.getSetState(),
                swerveModules.rearLeft.getSetState(),
                swerveModules.rearRight.getSetState()
        });
        SmartDashboard.putNumberArray("Swerve Module Distance", new double[] {
                swerveModules.frontLeft.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.frontRight.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.rearLeft.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.rearRight.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS});

        SmartDashboard.putNumberArray("Swerve Module Distance Revolutions", new double[] {
                swerveModules.frontLeft.getPosition().distanceMeters,
                swerveModules.frontRight.getPosition().distanceMeters,
                swerveModules.rearLeft.getPosition().distanceMeters,
                swerveModules.rearRight.getPosition().distanceMeters});
    }

    private void logModuleStates(String key, SwerveModuleState[] swerveModuleStates) {
        List<Double> dataList = new ArrayList<>();
        for (SwerveModuleState swerveModuleState : swerveModuleStates) {
            dataList.add(swerveModuleState.angle.getRadians());
            dataList.add(swerveModuleState.speedMetersPerSecond);
        }
        SmartDashboard.putNumberArray(key,
                dataList.stream().mapToDouble(Double::doubleValue).toArray());
    }

    public Rotation2d getRotation() {
        Rotation2d rot = gyro.getAngle();

        if (DriveConstants.GYRO_REVERSED) {
            rot = rot.unaryMinus();
        }

        return rot;
    }
    
    public Rotation3d getRotation3d() {
        return gyro.getRotation3d();
    }

    public SwerveModulePosition[] getModulePositions() {
        return swerveModules.getPositions().asArray();
    }

    /**
     * Method to drive the drivetrain using chassis speeds.
     *
     * @param speeds The chassis speeds.
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        swerveModules.setDesiredStates(
                new SwerveModules.States(
                        // front left
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        // front right
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        // rear left
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        // rear right
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                ).asArray()
        );
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        swerveModules.setDesiredStates(desiredStates);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        swerveModules.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate().getDegrees();
    }
   
    public ChassisSpeeds pgetChassisSpeed() {
       return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(swerveModules.frontLeft.getState(),swerveModules.frontRight.getState(),swerveModules.rearLeft.getState(),swerveModules.rearRight.getState());
    }
    public  Pose2d pgetEstimatedPose() {
        return PoseEstimation.pposeEstimator.getEstimatedPosition();
    }
    public void presetPose(Pose2d pose) {
        PoseEstimation.pposeEstimator.resetPosition(RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), pose);
    }

       public void BuilderConfigure(){
         AutoBuilder.configureHolonomic(
            this::pgetEstimatedPose, // Robot pose supplier
            this::presetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::pgetChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    3.0, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(  true,//Should the path be replanned at the start of path following if the robot is not already at the starting point?
                    false) //Should the path be replanned if the error grows too large or if a large error spike happens while following the path?
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
    );
        }
        
    
    
}
