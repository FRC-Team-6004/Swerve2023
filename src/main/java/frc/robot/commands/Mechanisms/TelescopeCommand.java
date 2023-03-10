package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopeCommand extends CommandBase {
    double spd;
    ArmSubsystem subsystem;
    boolean finish = false;
    public TelescopeCommand(ArmSubsystem armSubsystem, double speed){
        subsystem = armSubsystem;
        spd = speed;
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        subsystem.manuelTelescope(spd);

        if(spd<0){
            if(-subsystem.telescope.getSelectedSensorPosition()>300000){
                finish = true;
            }
        }
        else{
            if(-subsystem.telescope.getSelectedSensorPosition()<1000){
                finish = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return (finish);
    }
}
