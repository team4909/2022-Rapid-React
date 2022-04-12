package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import frc.lib.bioniclib.AutoRoutineBase;

public class TwoBallDisrupt extends AutoRoutineBase {

    public TwoBallDisrupt() {
        super();
        addCommands(
            
        );
    }

    @Override
    protected List<String> addTrajectories() {
        List<String> t = List.of("TarmacN-E", "E-FGSpit");
        return t;
    }
}