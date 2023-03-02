package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class PivotCommand extends CommandBase {
    double ang;
    ArmSubsystem subsystem;
    public PivotCommand(ArmSubsystem armSubsystem, double angle){
        subsystem = armSubsystem;
        ang = angle;
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        subsystem.setPivotPosition(ang);
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
