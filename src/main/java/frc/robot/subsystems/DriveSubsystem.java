package frc.robot.subsystems;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.config.ReplanningConfig;
// ... any other PathPlanner related classes


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem  extends SubsystemBase{
    private final AHRS navx; // Import AHRS from NavX library

    private CANSparkMax topLeft, bottomLeft, topRight, bottomRight, climberMotor;
    private PWMMotorController leftGroup = new PWMMotorController(null, 0) {
        
    };
    private DifferentialDrive difDrive;


    private RelativeEncoder leftEncoder, rightEncoder; // From REV library
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveWheelPositions wheelPos;

// ... other variables as needed

    public DriveSubsystem() {
        navx = new AHRS(SPI.Port.kMXP);

        // Motor Initialization (similar to your code) ... 

        // Encoder Setup & Odometry
        leftEncoder = topLeft.getEncoder();
        rightEncoder = topRight.getEncoder();
        // ... Encoder setup with conversion factors

        rightEncoder.setPositionConversionFactor((DriveTrainConstants.kLinearDistanceConversionFactor));
        leftEncoder.setPositionConversionFactor((DriveTrainConstants.kLinearDistanceConversionFactor));

        rightEncoder.setVelocityConversionFactor((DriveTrainConstants.kLinearDistanceConversionFactor/60));
        leftEncoder.setVelocityConversionFactor((DriveTrainConstants.kLinearDistanceConversionFactor/60));

        bottomLeft.follow(topLeft);
        bottomRight.follow(topRight);

        topRight.setInverted(true);
        topLeft.setInverted(true);


        navx.reset();
        //navx.
        resetEncoders();

        wheelPos = new DifferentialDriveWheelPositions(getLeftEncoderPosition(), getRightEncoderPosition());
        odometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        
        odometry.resetPosition(navx.getRotation2d(), wheelPos, new Pose2d());
        //odometry.resetPosition(new Pose2d(), navx.getRotation2d());
        // All other subsystem initialization
        // ...

        // Configure AutoBuilder last
        // AutoBuilder.configureRamsete(
        //         this::getPose, // Robot pose supplier
        //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        //         this::drive, // Method that will drive the robot given ChassisSpeeds
        //         new ReplanningConfig(), // Default path replanning config. See the API for the options here
        //         () -> {
        //         // Boolean supplier that controls when the path will be mirrored for the red alliance
        //         // This will flip the path being followed to the red side of the field.
        //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        // );
    }
    public void resetEncoders(){
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public void arcadeDrive(double fwd, double rot){
        difDrive.arcadeDrive(fwd, rot);
    }

    public double getRightEncoderPosition(){
        return rightEncoder.getPosition();
    }
    public double getLeftEncoderPosition(){
        return leftEncoder.getPosition();
    }

    public double getRightEncoderVelocity(){
        return rightEncoder.getVelocity();
    }
    public double getLeftEncoderVelocity(){
        return leftEncoder.getVelocity();
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public void resetPose(Pose2d pose){
        resetEncoders();
        odometry.resetPosition(navx.getRotation2d(),leftEncoder.getPosition(),rightEncoder.getPosition(),pose);

    }
    public DifferentialDriveWheelSpeeds getCurrentSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(),getRightEncoderVelocity());
    }

    @Override
    public void periodic(){
        odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition()); 
    }


    
}
