// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import edu.wpi.first.math.util.Units;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  public static final Joystick joystick1 = new Joystick(0);
  public static final Joystick joystick2 = new Joystick(IntakeConstants.AngleController);
  
  public static final Drivetrain drivetrain = new Drivetrain();

  public static final PoseEstimation poseEstimation = new PoseEstimation();

  public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();

  public static Field2d field = new Field2d();
  public static Field2d nodeSelector = new Field2d();

  private final FieldObject2d startingPosition = field.getObject("Starting Position");
  private final FieldObject2d autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position");

  private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystick1);
  private AutoBalance autoBalanceCommand = new AutoBalance(drivetrain);
 
  public static SendableChooser<String> autoSelector;
  private final SendableChooser<Command> autoChooser;

  //private static AutoElevator autoElevator;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     autoChooser = AutoBuilder.buildAutoChooser("New Auto");
  SmartDashboard.putData("Auto Mod", autoChooser);
    
     NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));



  drivetrain.setDefaultCommand(driveCommand);

  if (autoBalanceStartingPosition.getPoses().isEmpty()) {
    autoBalanceStartingPosition.setPose(AllianceUtils.allianceToField(new Pose2d(new Translation2d(0,0),new Rotation2d())));
  }

    configureBindings();

  }

  

  

 
  private void configureBindings() {

  
      // Pose Estimation
      
      new JoystickButton(joystick1, 6)
              .onTrue(new InstantCommand(driveCommand::resetFieldOrientation));
      new JoystickButton(joystick1, 7)
              .onTrue(new InstantCommand(() -> poseEstimation.resetPose(
                      new Pose2d(
                              poseEstimation.getEstimatedPose().getTranslation(),
                              new Rotation2d())))); 

      // Driving 
       
      new JoystickButton(joystick1, 1)
              .whileTrue(new RunCommand(
                      drivetrain::setX,
                      drivetrain)); 


      new JoystickButton(joystick1, 3)
              .whileTrue(autoBalanceCommand);

   SmartDashboard.putData("Example Auto", new PathPlannerAuto("new Auto"));

   SmartDashboard.putData("Pathfind to 1 Pos", AutoBuilder.pathfindToPose(
    new Pose2d(2.70, 2.01, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      3.0, 3.0, 
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    ), 
    0, 
    2.67
  ));

  SmartDashboard.putData("Pathfind to 2 Pos", AutoBuilder.pathfindToPose(
    new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    ), 
    0, 
    3.35
  ));

  // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = poseEstimation.getEstimatedPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          3.0, 3.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  
  }
   

  public Command getAutonomousCommand() {
    Pose2d startingPose = startingPosition.getPose();
    //PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path 1");
    return new SequentialCommandGroup(
    new InstantCommand(() -> poseEstimation.resetPose(startingPose)),
    new InstantCommand(() -> poseEstimation.resetPose(startingPose)),
     autoChooser.getSelected()
    );
//autoChooser.getSelected().andThen(AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3, 3, 540, 720)));
  }

}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

