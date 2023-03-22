package frc.robot.commands.Mechanisms;

import java.sql.Time;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DrivetrainTimerCommand extends CommandBase {
    ChassisSpeeds speed;
    SwerveSubsystem subsystem;
    Timer timer;
    double time;
    public DrivetrainTimerCommand(SwerveSubsystem swerveSubsystem, ChassisSpeeds chassisSpeeds, double timeEnd){
        subsystem = swerveSubsystem;
        speed = chassisSpeeds;
        time = timeEnd;
    }

    @Override
    public void initialize() {
        //
        timer.start();
    }

    @Override
    public void execute() {
        subsystem.move(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.move(new ChassisSpeeds(0,0,0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (timer.get() >= time);
    }
}
