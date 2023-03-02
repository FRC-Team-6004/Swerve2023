package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class GrabCommand extends CommandBase {
    boolean grb;
    ArmSubsystem subsystem;
    public GrabCommand(ArmSubsystem armSubsystem, boolean grab){
        subsystem = armSubsystem;
        grb = grab;
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        subsystem.setGrab(grb);
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
