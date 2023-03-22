package frc.robot.commands.Mechanisms;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceCommand extends CommandBase {
    SwerveSubsystem subsystem;
    double angle;
    boolean onChargeStation = false;
    boolean finished = false;
    public BalanceCommand(SwerveSubsystem swerveSubsystem){
        subsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        angle = subsystem.gyro.getPitch();
        subsystem.balance();
        /*
        if(Math.abs(angle) < 2.5 && !onChargeStation){
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
            subsystem.setModuleStates(moduleStates);
        }
        else if(Math.abs(angle) < 2.5 && onChargeStation){
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));
            subsystem.setModuleStates(moduleStates);
            finished = true;
        }
        else {
            subsystem.balance();
            onChargeStation = true;
        }
        */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));
        subsystem.setModuleStates(moduleStates);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(angle)<2.5);
    }
}
