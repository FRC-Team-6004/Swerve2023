// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.Mechanisms.GrabCommand;
import frc.robot.commands.Mechanisms.PivotCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public ArmSubsystem armSubsystem = new ArmSubsystem();

  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
        swerveSubsystem,
        armSubsystem,
        () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> driverController.getRawButton(OIConstants.kDriverBrakeButton),
        () -> driverController.getRawButton(OIConstants.kAlignWithTargetButton),
        () -> driverController.getRawButton(OIConstants.kResetDirectionButton),

        () -> operatorController.getRawButton(OIConstants.kRotate0Button),
        () -> operatorController.getRawButton(OIConstants.kRotate180Button),
        () -> operatorController.getRawButton(OIConstants.kExtendFullButton),
        () -> operatorController.getRawButton(OIConstants.kRetractButton),
        () -> operatorController.getRawButton(OIConstants.kToggleGrabButton),
        () -> operatorController.getRawButton(OIConstants.kReverseGrabButton),
        () -> operatorController.getRawButton(OIConstants.kForwardGrabButton),
        () -> operatorController.getRawButton(OIConstants.kManuelButton)
        ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //new JoystickButton(driverController, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    swerveSubsystem.brake(true);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kDriveKinematics);
    Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(Math.PI));
    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            initialPose,
            List.of(
                    new Translation2d(0, 0),
                    new Translation2d(1, 0)),
            new Pose2d(2, 0, new Rotation2d(Math.PI)),
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
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);
    swerveSubsystem.resetOdometry(initialPose);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
            new InstantCommand(() -> armSubsystem.setPivotPosition(45)),
            new WaitCommand(2),
            new InstantCommand((() -> armSubsystem.setGrab(true))),
            new WaitCommand(2),
            
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0,0,new Rotation2d(Math.PI)))),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsystem.stopModules()));

}
}

