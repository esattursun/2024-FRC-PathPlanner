//10 PARÇA OTONOM

// 10 kere 1 parça al
// speakera git 
// 10 kere 1 parça at 
package frc.robot.commands.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class FRCPathPlanner {
    public static final SendableChooser<Command> pathChooser=new SendableChooser<>();
   
    

    public final static SendableChooser<Command> autoChooser=AutoBuilder.buildAutoChooser("New Auto");
    
    public static void SetPathPlannerSettings(){
      setSmartDashboard();
      CommandNameEntry();
      addPathOptions();
      addAutoOptions();
      
    }
    public static void setSmartDashboard(){
        SmartDashboard.putData("Path Mod",pathChooser);
        SmartDashboard.putData("Auto Mod", autoChooser);
        SmartDashboard.putBoolean("is AutoBuilder configure?", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean(" is pathfinding configure?", AutoBuilder.isPathfindingConfigured());
    }

    public static void addPathOptions(){
      //  pathChooser.addOption("Path 1", Commands.runOnce(() -> {followPathCommand(Example Path 1);} ));
      // pathChooser.addOption("Path 2", Commands.runOnce(() -> {followPathCommand(Example Path 2);} ));
         pathChooser.addOption("nothing",Commands.none());
         
        try (Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner","paths"))) {//deploy/patplanner/paths klasörünün içindekilerinin alınması
      files.filter(file -> !Files.isDirectory(file))//sadece dosyaların alınması
          .map(Path::getFileName)//dosya adlarının alınması
          .map(Path::toString)//dosya adlarının stringe çevrilmesi
          .filter(fileName -> fileName.endsWith(".path"))// Sadece ".path" uzantılı dosyaların seçilmesi.
          .sorted()//seçilernlerin sıralanması
          .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))//sadece noktadan önceki kısmı alır
          .forEach(pathName -> pathChooser.addOption("PP:" + pathName,Commands.runOnce(() -> {followPathCommand(pathName);})));
    } catch (IOException e) {
      System.out.println("********* Failed to list PathPlanner paths. *********");
    }

    }

    public static void addAutoOptions(){
    autoChooser.setDefaultOption("nothing", Commands.none());
    /*autoChooser.addOption("Example Auto", new PathPlannerAuto("new Auto"));
     autoChooser.addOption("Example Auto2", new PathPlannerAuto("new Auto2"));
     */
     
	  /*  // create autos
     final List<String> autoNames = AutoBuilder.getAllAutoNames();
    
  		for (final String autoName : autoNames) {
  			final Command auto = new PathPlannerAuto(autoName);
  			autoChooser.addOption(autoName, auto);
  		} we don't need this because the buildautochooser function already does this.
    */
    
   
    }

    public static void CommandNameEntry(){
    NamedCommands.registerCommand("intake", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("shoot", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    //NamedCommands.registerCommand("intake", new SequentialCommandGroup(new IntakeCommand(ShooterSubsystem,1)));
    //NamedCommands.registerCommand("shoot", new SequentialCommandGroup(new ShootCommand(ShooterSubsystem,1))););
    }

    
    public static Command followPathCommand(String pathName) {
     PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return new FollowPathHolonomic(
         path,
         RobotContainer.poseEstimation::getEstimatedPose, // Robot pose supplier
         RobotContainer.drivetrain::pgetChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        RobotContainer.drivetrain::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                 new PIDConstants(ModuleConstants.DRIVING_P, ModuleConstants.DRIVING_I, ModuleConstants.DRIVING_D), // Translation PID constants
                 new PIDConstants(ModuleConstants.TURNING_P, ModuleConstants.TURNING_I, ModuleConstants.TURNING_D), // Rotation PID constants
                 DriveConstants.MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
                 DriveConstants.WHEEL_BASE/2, // Drive base radius in meters. Distance from robot center to furthest module.
                 new ReplanningConfig() //Should the path be replanned if the error grows too large or if a large error spike happens while following the path?
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
       RobotContainer.drivetrain // Reference to this subsystem to set requirements
        );
    }
    
}
