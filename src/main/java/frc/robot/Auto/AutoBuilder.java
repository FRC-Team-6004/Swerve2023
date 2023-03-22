package frc.robot.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Mechanisms.BalanceCommand;
import frc.robot.commands.Mechanisms.DrivetrainCommand;
import frc.robot.commands.Mechanisms.TelescopeCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;
    private SequentialCommandGroup autoCommand;
    //private AutoPath startPath;

    public Command build() {

        autoCommand = new SequentialCommandGroup();

        switch (autoMode) {
            case AUTOSIDES:
                autoSides();
                break;
            case AUTOCHARGESTATION:
                autoChargeStation();
                break;

        }

        //autoCommand.beforeStarting(startPath.odometryReset());
        return autoCommand;
    }

    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    private void auto1() {
        /*
        //startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO1");
        autoCommand.addCommands(
            new SequentialCommandGroup(
                
                //startPath.zeroHeading(),
                //startPath.setBrake(),
                //startPath.odometryReset(),
                //startPath.getAutoPath()
                //new GrabCommand(robotContainer.armSubsystem, false) //this how to add command
                new DrivetrainCommand(robotContainer.swerveSubsystem, new ChassisSpeeds(1.5, 0, 0)),
                new WaitCommand(2), 
                new DrivetrainCommand(robotContainer.swerveSubsystem, new ChassisSpeeds(0, 0, 0))
                

            )
        );
        */
    }
    private void autoSides() {
        SmartDashboard.putString("autonomous", "Sides");

        robotContainer.swerveSubsystem.brake(true);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, 0),
                        new Translation2d(1, 0)), //half for some reason
                new Pose2d(2.0, 0, new Rotation2d(0)), //half for some reason
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                robotContainer.swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                robotContainer.swerveSubsystem::setModuleStates,
                robotContainer.swerveSubsystem);
        //startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO1");
        autoCommand.addCommands(
            new SequentialCommandGroup(  
                
                new InstantCommand(() -> robotContainer.armSubsystem.setPivotPosition(-53.5)),
                new WaitCommand(1.5),
                new TelescopeCommand(robotContainer.armSubsystem, -.9),
                //new InstantCommand(() -> armSubsystem.setTelescopePosition(0.9)),
                new WaitCommand(1.5),
                new InstantCommand((() -> robotContainer.armSubsystem.setGrab(true))),
                new WaitCommand(0.5),
                new InstantCommand(() -> robotContainer.armSubsystem.setPivotPosition(0)),
                //new InstantCommand(() -> armSubsystem.setTelescopePosition(0.1)),
                new WaitCommand(1.5),
                new TelescopeCommand(robotContainer.armSubsystem, .9),
                new WaitCommand(1.5),
                
                new InstantCommand(() -> robotContainer.swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                swerveControllerCommand,
                new InstantCommand(() -> robotContainer.swerveSubsystem.stopModules()),
                new WaitCommand(2)
            )
        );
    }

    private void autoChargeStation() {
        SmartDashboard.putString("autonomous", "Sides");

        robotContainer.swerveSubsystem.brake(true);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, 0),
                        new Translation2d(1, 0)), //half for some reason
                new Pose2d(1.25, 0, new Rotation2d(0)), //half for some reason
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                robotContainer.swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                robotContainer.swerveSubsystem::setModuleStates,
                robotContainer.swerveSubsystem);
        //startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO1");
        autoCommand.addCommands(
            new SequentialCommandGroup(  
                
                /*new InstantCommand(() -> robotContainer.armSubsystem.setPivotPosition(-53.5)),
                new WaitCommand(1.5),
                new TelescopeCommand(robotContainer.armSubsystem, -.9),
                //new InstantCommand(() -> armSubsystem.setTelescopePosition(0.9)),
                new WaitCommand(1.5),
                new InstantCommand((() -> robotContainer.armSubsystem.setGrab(true))),
                new WaitCommand(0.5),
                new InstantCommand(() -> robotContainer.armSubsystem.setPivotPosition(0)),
                //new InstantCommand(() -> armSubsystem.setTelescopePosition(0.1)),
                new WaitCommand(1.5),
                new TelescopeCommand(robotContainer.armSubsystem, .9),
                new WaitCommand(1.5) , */
                
                new InstantCommand(() -> robotContainer.swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                swerveControllerCommand,
                new InstantCommand(() -> robotContainer.swerveSubsystem.stopModules()),
                new WaitCommand(0.25),
                new BalanceCommand(robotContainer.swerveSubsystem)
                
            )
        );
    }


}