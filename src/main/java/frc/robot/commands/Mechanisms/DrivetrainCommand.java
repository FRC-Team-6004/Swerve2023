package frc.robot.commands.Mechanisms;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DrivetrainCommand extends CommandBase {
    ChassisSpeeds speed;
    SwerveSubsystem subsystem;
    public DrivetrainCommand(SwerveSubsystem swerveSubsystem, ChassisSpeeds chassisSpeeds){
        subsystem = swerveSubsystem;
        speed = chassisSpeeds;
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        subsystem.setModuleStates(moduleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
