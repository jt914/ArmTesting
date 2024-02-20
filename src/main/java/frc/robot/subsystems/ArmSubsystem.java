package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;


public class ArmSubsystem extends ProfiledPIDSubsystem {

    private CANSparkMax armLeft,armRight;
    public DutyCycleEncoder armEnc;

    private static double kP = 0.4;
    private static double kI = .05;;
    private static double kD = 0.00008;
    private static PIDController controller = new PIDController(kP, kI, kD);

    private static double maxVelocity = 100; // degrees per second
    private static double maxAcceleration = 200; // degrees per seconds^2
    private static ProfiledPIDController profiledPIDController = new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                maxVelocity, // degrees per second
                maxAcceleration) // degrees per second^2
                , 0.02);
                
    private static double kS = 0.01;
    private static double kG = 0.22;
    private static double kV = 0;
    private ArmFeedforward feedForward = new ArmFeedforward(kS, kG, kV);

    private static double iZone = 1;
    private static double clamp = 6;

    public ArmSubsystem() {
        super(profiledPIDController);
        
        System.out.println("armsub");
        SmartDashboard.putData("ArmPidController", profiledPIDController);
        SmartDashboard.putNumber("kS", kS);
        SmartDashboard.putNumber("kG", kG);
        SmartDashboard.putNumber("kV", kV);

        SmartDashboard.putNumber("iZone", 1);
        SmartDashboard.putNumber("clamp", 5);

        SmartDashboard.putNumber("maxVelocity", maxVelocity);
        SmartDashboard.putNumber("maxAcceleration", maxAcceleration);
    
        getController().setTolerance(0.2);
        
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kBrake);
        armRight.enableVoltageCompensation(11);
        armRight.setInverted(true);

        armRight.burnFlash();

        armEnc = new DutyCycleEncoder(9); 
        armEnc.setPositionOffset(0.513);
        armEnc.setDistancePerRotation(-360);

        setGoal(10);
    }

    @Override
    protected double getMeasurement() {
        return armEnc.getDistance();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {

        iZone = SmartDashboard.getNumber("iZone", 1);
        kS = SmartDashboard.getNumber("kS", kS);
        kG = SmartDashboard.getNumber("kG", kG);
        kV = SmartDashboard.getNumber("kV", kV);
        SmartDashboard.putNumber("output", output);

        controller.setIZone(iZone);

        maxVelocity = SmartDashboard.getNumber("maxVelocity", 0);
        maxAcceleration = SmartDashboard.getNumber("maxAcceleration", 0);
        TrapezoidProfile.Constraints tpc = new TrapezoidProfile.Constraints(
                maxVelocity, // degrees per second
                maxAcceleration); // degrees per second^2
        
            profiledPIDController.setConstraints(tpc);

        // Feed Forward
        double direction = Math.signum(setpoint.position - armEnc.getDistance());
        feedForward = new ArmFeedforward(kS, kG, kV);
        double ffOutput = feedForward.calculate(setpoint.position * Math.PI / 180, direction * Math.PI/10);
        
        SmartDashboard.putNumber("ffOutput", ffOutput);
        SmartDashboard.putNumber("encoderValue", armEnc.getDistance());

        // PID output + Feed Forward output
        output = output + ffOutput;

        // Limit output voltage
        clamp = SmartDashboard.getNumber("clamp", 1);
        output = MathUtil.clamp(output, -clamp, clamp);

        // Set output voltage
        armLeft.setVoltage(output);
        armRight.setVoltage(output);

    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
      }


    
}