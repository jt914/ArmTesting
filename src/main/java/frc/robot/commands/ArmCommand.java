package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {

    private final ArmSubsystem armSubsystem;

    public ArmCommand(ArmSubsystem _armSubsystem) {
        armSubsystem = _armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute(){

        SmartDashboard.putData(armSubsystem);

        double currentSetpoint = armSubsystem.getController().getGoal().position;
        
        if (Constants.swerveController.getHID().getLeftTriggerAxis() > .5) {
            currentSetpoint = 10;
        }
        else if(Constants.swerveController.getHID().getRightBumperPressed()){
            currentSetpoint = 20;
        }
        else if(Constants.swerveController.getHID().getLeftBumperPressed()){
            currentSetpoint = 30;
        }
        else if(Constants.swerveController.getHID().getRightTriggerAxis() > .5){
            currentSetpoint = 35;
        }
        currentSetpoint = MathUtil.clamp(currentSetpoint, 10,110);
        armSubsystem.setGoal(currentSetpoint);

    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.disable();
    }






}