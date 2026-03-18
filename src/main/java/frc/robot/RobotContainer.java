//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;

// import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
// import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    public static double velocityYAmount;
    public static double velocityXAmount;
    public static double rotationAmount;
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
    private double slowdown = 1;
    private boolean slowToggled = false;
    private double MaxSpeed = slowdown*TunerConstants.kSpeedAt12VoltsFront.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double slowdownMult = 0.8;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private SwerveDrivetrainConstants drivetrainconstants = TunerConstants.DrivetrainConstants;
    private Pigeon2 gyro = new Pigeon2(drivetrainconstants.Pigeon2Id);

    private double logVelocityXAmount(double amount){
        velocityXAmount = amount;
        return amount;
    }

    private double logVelocityYAmount(double amount){
        velocityYAmount = amount;
        return amount;
    }

    private double logRotationAmount(double amount){
        rotationAmount = amount;
        return amount;
    }


    public RobotContainer() {

        configureBindings();

    }

    private void slowDown(){
        slowdown-=0.02;
        if (slowdown < 0.2){
            slowdown=0.2;
        }

    }
    private void speedUp(){
        slowdown+=0.02;
        if (slowdown > 1){
            slowdown=1;
        }
    }
    private double fieldOrentedX(double speedx, double speedy){
        double sinspeedx = speedx*Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
        double cosspeedx = speedx*-Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getCos());
        double sinspeedy = speedy*-Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
        double cosspeedy = speedy*Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getCos());

        double cosSpeed = -Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getCos());
        double sinSpeed = Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
        double tcosSpeed = RobotContainer.drivetrain.getState().Pose.getRotation().getCos();
        double tsinSpeed = RobotContainer.drivetrain.getState().Pose.getRotation().getSin();


        double dflip=-(1-Math.abs(tsinSpeed-tcosSpeed));

        double flip = dflip*(cosspeedy-sinspeedy)*Math.abs(1-Math.abs(sinSpeed+cosSpeed));

        double finSpeedx =sinspeedx + cosspeedx+flip;





        return finSpeedx;
    }
    private double fieldOrentedY(double speedy, double speedx){
        double sinspeedy = speedy*-Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
        double cosspeedy = speedy*Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getCos());
        double sinspeedx = speedx*Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
        double cosspeedx = speedx*-Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getCos());
        
        double cosSpeed = Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getCos());
        double sinSpeed = -Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
        double tcosSpeed = RobotContainer.drivetrain.getState().Pose.getRotation().getCos();
        double tsinSpeed = RobotContainer.drivetrain.getState().Pose.getRotation().getSin();
    

        double dflip=-(1-Math.abs(tsinSpeed-tcosSpeed));
        
        double flip = dflip*(sinspeedx-cosspeedx)*Math.abs(1-Math.abs(cosSpeed+sinSpeed));


        double finSpeedy =sinspeedy + cosspeedy+flip;




        return finSpeedy;
    }
    
    private double trueY() {
        if (!slowToggled)
        return joystick.getLeftY();
        else {
            double returnValue = joystick.getLeftY()*slowdownMult;
            return (returnValue);
        }
    }
    private double trueX() {
        if (!slowToggled)
        return joystick.getLeftX();
        else {
            double returnValue = joystick.getLeftX()*slowdownMult;
            return (returnValue);
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically 
            //usually trueX/trueY is replaced with joystick.getLeftX/getLeftY
            drivetrain.applyRequest(() ->
                drive.withVelocityY(logVelocityYAmount( fieldOrentedY(-trueY(),trueX()+(-joystick.getRightX()*0.01))* MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityX(logVelocityXAmount(fieldOrentedX(trueX()+(-joystick.getRightX()*0.01),-trueY())* MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(logRotationAmount(-joystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );
        // While the left bumper on operator controller is held, intake Fuel
    joystick.x().whileTrue(new Intake(fuelSubsystem));
    // While the y button on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    joystick.y().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    joystick.a().whileTrue(new Eject(fuelSubsystem));


    // climber up/down controls
    joystick.povDown().whileTrue(new ClimbDown(climberSubsystem));
    joystick.povUp().whileTrue(new ClimbUp(climberSubsystem));


    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        

        //slow down the robot 
        joystick.b().onTrue(drivetrain.runOnce(() -> {
            slowToggled = !slowToggled;
        }));

        // Reset the field-centric heading on right DPAD press.
        joystick.povRight().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            System.out.println("Oriented");
        }));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
