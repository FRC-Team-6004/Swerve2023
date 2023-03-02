package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopeCommand extends CommandBase {
    double pos;
    ArmSubsystem subsystem;
    public TelescopeCommand(ArmSubsystem armSubsystem, double position){
        subsystem = armSubsystem;
        pos = position;
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        subsystem.setTelescopePosition(pos);
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
