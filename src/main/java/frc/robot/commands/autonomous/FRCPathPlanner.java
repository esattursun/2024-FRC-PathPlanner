package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FRCPathPlanner {
    public static final SendableChooser<Command> pathChooser=new SendableChooser<>();
    private static final Drivetrain drivetrain = new Drivetrain();
    private static final PoseEstimation poseEstimation = new PoseEstimation();

    public final static SendableChooser<Command> autoChooser=AutoBuilder.buildAutoChooser("New Auto");

    public static void setSmartDashboard(){

        SmartDashboard.putData("Path Mod",pathChooser);
        SmartDashboard.putData("Auto Mod", autoChooser);
        SmartDashboard.putBoolean("İs AutoBuilder configure?", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean(" is pathfinding configure?", AutoBuilder.isPathfindingConfigured());
    }

    public static void FindPath(){

    pathChooser.addOption("Pathfind to 1 Pos", AutoBuilder.pathfindToPose(
    new Pose2d(2.70, 2.01, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      3.0, 3.0, 
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    ), 
    0, 
    2.67
    ));

    pathChooser.addOption("Pathfind to 2 Pos", AutoBuilder.pathfindToPose(
    new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    new PathConstraints(
      3.0, 3.0, 
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    ), 
    0, 
    3.35
    ));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    pathChooser.addOption("On-the-fly path", Commands.runOnce(() -> {
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
  
    public static void FollowPath(){
        pathChooser.addOption("Path 1", Commands.runOnce(() -> {followPathCommand("Example Path 1");} ));
        pathChooser.addOption("Path 2", Commands.runOnce(() -> {followPathCommand("Example Path 2");} ));
    }

    public static void addAutoOptions(){

    // autoChooser.addOption("Example Auto", new PathPlannerAuto("new Auto"));
    //autoChooser.addOption("Example Auto2", new PathPlannerAuto("new Auto2"));
    // autoChooser.addOption("Example Auto3", new PathPlannerAuto("new Auto3"));
    // autoChooser.addOption("Example Auto4", new PathPlannerAuto("new Auto4"));

     // create paths
		autoChooser.addOption("nothing", Commands.none());
		final List<String> autoNames = AutoBuilder.getAllAutoNames();
    
		for (final String autoName : autoNames) {
			final Command auto = new PathPlannerAuto(autoName);
			autoChooser.addOption(autoName, auto);
		}
    }

    public static void CommandNameEntry(){
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    //NamedCommands.registerCommand("intake", new SequentialCommandGroup(new IntakeCommand(ShooterSubsystem,1)));
    //NamedCommands.registerCommand("shoot", new SequentialCommandGroup(new ShootCommand(ShooterSubsystem,1))););
    }

    public static void ConfigurePathPlanner(){
        AutoBuilder.configureHolonomic(
            PoseEstimation::pgetEstimatedPose, // Robot pose supplier
            PoseEstimation::presetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::pgetChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drivetrain::drive,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    3.0, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
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
            drivetrain // Reference to this subsystem to set requirements
    );
    }
    
    public static Command followPathCommand(String pathName) {
     PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return new FollowPathHolonomic(
         path,
         PoseEstimation::pgetEstimatedPose, // Robot pose supplier
         drivetrain::pgetChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
         drivetrain::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                 3.0, // Max module speed, in m/s
                 0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                 new ReplanningConfig() // Default path replanning config. See the API for the options here
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
         drivetrain // Reference to this subsystem to set requirements
        );
    }
}
