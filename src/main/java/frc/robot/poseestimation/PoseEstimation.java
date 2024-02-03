package frc.robot.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class PoseEstimation {
    
    private SwerveDrivePoseEstimator poseEstimator;
public static SwerveDrivePoseEstimator pposeEstimator; 


   

    private TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5);
  private static TimeInterpolatableBuffer<Pose2d> pposeHistory = TimeInterpolatableBuffer.createBuffer(1.5);

    private static final double DIFFERENTIATION_TIME = Robot.kDefaultPeriod;

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            RobotContainer.drivetrain.getRotation(),
            RobotContainer.drivetrain.getModulePositions(),
            new Pose2d(),
            Constants.DriveConstants.ODOMETRY_STD_DEV,
            VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );

  

        pposeEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            RobotContainer.drivetrain.getRotation(),
            RobotContainer.drivetrain.getModulePositions(),
            new Pose2d(),
            Constants.DriveConstants.ODOMETRY_STD_DEV,
            VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );

        

    }

    public void periodic() {
        poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
        pposeHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
      
        

        RobotContainer.field.setRobotPose(getEstimatedPose());
    }




    public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
     

    public Translation2d getEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = poseHistory.getSample(now).get().getTranslation();
        Translation2d previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).get().getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }
     public static Translation2d pgetEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = pposeHistory.getSample(now).get().getTranslation();
        Translation2d previous = pposeHistory.getSample(now - DIFFERENTIATION_TIME).get().getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), pose);
    }
}