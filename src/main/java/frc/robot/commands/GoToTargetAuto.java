package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.commands.TurnToAngle;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GoToTargetAuto extends CommandGroup {

    public GoToTargetAuto() {
        double speed = 0.8;
        double x = SmartDashboard.getNumber("X", 0.0);
        double z = SmartDashboard.getNumber("Z", 0.0);
        double angle = SmartDashboard.getNumber("Angle", 0.0);
        addSequential(new TurnToAngle(angle - 90, 1000));
        addSequential(new DriveForward(x));
        addSequential(new TurnToAngle(90, 1000));
        addSequential(new DriveForward(z));
    }
}

