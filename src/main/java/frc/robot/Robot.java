package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.Timer;

// Main robot class
public class Robot extends TimedRobot {

    // Constants
    private static final int ECHO_CHANNEL = 0; // Ultrasonic echo channel
    private static final int PING_CHANNEL = 1; // Ultrasonic ping channel

    // Ultrasonic sensor
    private Ultrasonic ultrasonicSensor;

    // State variables
    private int autonomousState;
    private Timer timer;

    // Input devices
    private Joystick joystick;
    private PS4Controller ps4Controller;
    //private AHRS navx;

    // Drive speed
    private double driveSpeed = 0.8;

    // PhotonVision
    private PhotonCamera camera1, camera2;

    // Motor controllers
    private CANSparkMax topLeftDrive, bottomLeftDrive, topRightDrive, bottomRightDrive;
    private CANSparkMax climberMotor, intakeMotor, shooterUpMotor, shooterDownMotor, armMotor1 /*armMotor2*/;
    private RelativeEncoder armEncoder,armEncoder2; // For arm position tracking
    private MecanumDrive mecanumDrive;
    private boolean DEFAULT= false;
    private boolean FEED= false;
    private boolean SHOOT= false;
    private boolean SPEAKER= false;
    private boolean acc = false;


    @Override
    public void robotInit() {

        // Initialize NetworkTables and PhotonVision
       // camera1 = new PhotonCamera("6213-ringcam"); // Replace with your camera's name in PhotonVision
        ps4Controller = new PS4Controller(1);
        timer= new Timer();

        // Initialize input devices
        //navx = new AHRS(edu.wpi.first.wpilibj.SerialPort.Port.kUSB1);
        joystick = new Joystick(0);
        ps4Controller = new PS4Controller(1);

        // Initialize ultrasonic sensor
        ultrasonicSensor = new Ultrasonic(PING_CHANNEL, ECHO_CHANNEL);
        Ultrasonic.setAutomaticMode(true); // Enable automatic ranging

        // Initialize motor controllers
        topLeftDrive = new CANSparkMax(3, MotorType.kBrushless);
        bottomLeftDrive = new CANSparkMax(2, MotorType.kBrushless);
        topRightDrive = new CANSparkMax(4, MotorType.kBrushless);
        bottomRightDrive = new CANSparkMax(1, MotorType.kBrushless);
       // climberMotor = new CANSparkMax(10, MotorType.kBrushless);
        //intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
        //shooterUpMotor = new CANSparkMax(7, MotorType.kBrushless);
        //shooterDownMotor = new CANSparkMax(8, MotorType.kBrushless);
        armMotor1 = new CANSparkMax(7, MotorType.kBrushless);
        //armMotor2 = new CANSparkMax(6, MotorType.kBrushless);

        // Configure motor inversions
        
       // armEncoder = armMotor1.getEncoder();

        for(int i=0; i<5; i++){
       // armEncoder.setPosition(0);
        topLeftDrive.setInverted(true);
        topRightDrive.setInverted(false);
        bottomRightDrive.setInverted(false);
        bottomLeftDrive.setInverted(true);
        new WaitCommand(.1);
        }
        topLeftDrive.burnFlash();
        topRightDrive.burnFlash();
        bottomLeftDrive.burnFlash();
        bottomRightDrive.burnFlash();
        // Initialize MecanumDrive
        mecanumDrive = new MecanumDrive(topLeftDrive, bottomLeftDrive, topRightDrive, bottomRightDrive);

        // Initialize arm encoder
        

        // armEncoder2 = armMotor2.getEncoder();
        // armEncoder2.setPosition(0);
    }

    @Override
    public void autonomousInit() {
        timer.reset();
       timer.start();
        //autonomousState = 1;
    }

    @Override
    public void autonomousPeriodic() {
         //mecanumDrive.driveCartesian(0.0, 0.4, 0.0);
        if (timer.get() < 4.0) {
            // Drive forward with mecanum drive (0.0 is no strafe, 0.5 is half speed forward, 0.0 is no rotation)
            mecanumDrive.driveCartesian(0.3, 0.0, 0.0);
        } else {
            // Stop the robot after 4 seconds
            mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
            timer.stop();
        }
    }

    // @Override
    // public void autonomousPeriodic() {
        // if (timer.get() < 4.0) {
        //     // Drive forward with mecanum drive (0.0 is no strafe, 0.5 is half speed forward, 0.0 is no rotation)
        //     mecanumDrive.driveCartesian(0.0, 0.4, 0.0);
        // } else {
        //     // Stop the robot after 4 seconds
        //     mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
        //     timer.stop();
        // }
       
        // PhotonPipelineResult result = camera1.getLatestResult();
        // PhotonPipelineResult ressult2 = camera2.getLatestResult();
        // ArmPID();
        // shooterUpMotor.set(0.8);
        // shooterDownMotor.set(0.8);
        

        //worst case
        //  if (armEncoder.getPosition() > Constants.ShootConstants.SPEAKER-2 && armEncoder.getPosition() < Constants.ShootConstants.SPEAKER+2) {
                            
        //                     /* Shoot */
        //                     intakeMotor.set(1);
           
    
                        
        //                 }


        // switch (autonomousState) {
        //     case 1:
        //         // Turn 5 degrees, shoot, then turn back 5 degrees
        //         // navx.reset();
        //         SHOOT= true;
        //         FEED=DEFAULT=SPEAKER = false;
        //         if (!result2.hasTargets()) {
        //             mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
        //         }else{
        //             mecanumDrive.stopMotor();
        //             double targetYaw = result.getBestTarget().getYaw();
        //             if (targetYaw > -1 && targetYaw < 1) {
        //                 mecanumDrive.stopMotor();
        //                 if (armEncoder.getPosition() > Constants.ShootConstants.SPEAKER-2 && armEncoder.getPosition() < Constants.ShootConstants.SPEAKER+2) {
                            
        //                     /* Shoot */
        //                     intakeMotor.set(1);
        //                     if (!isRingClose()){
        //                         autonomousState = 2;
        //                         intakeMotor.set(.5);
        //                     }
                        
        //                 }

        //             } else {
        //                 mecanumDrive.driveCartesian(0.0, 0.0, 0.1);
        //             }

        //         } 
                
        //         break;

        //     case 2:
        //         // Move forward until ultrasonic sensor detects something close, then turn and shoot
        //         FEED= true;
        //         DEFAULT=SHOOT=SPEAKER = false;
        //         if (armEncoder.getPosition() > Constants.ShootConstants.FEED-2 && armEncoder.getPosition() < Constants.ShootConstants.FEED+2) {

        //             if (!result.hasTargets() && !isRingClose()) {
        //                 mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
        //             } else {
        //                 double targetYaw = result.getBestTarget().getYaw();
        //                 if (targetYaw > -1 && targetYaw < 1) {
        //                     mecanumDrive.driveCartesian(.5, 0.0, 0.0);
        //                 } else {
        //                     mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
        //                 }
        //             }
        //             if (isRingClose()) {
        //                 mecanumDrive.stopMotor();
        //                 autonomousState=1;
        //             } 
        //         }
        //         break;
            // case 3:
            //     // Move forward until ultrasonic sensor detects something close, then turn and shoot
            //     if (!result.hasTargets() && !isRingClose()) {
            //         mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
            //     } else {
            //         double targetYaw = result.getBestTarget().getYaw();
            //         if (targetYaw > -1 && targetYaw < 1) {
            //             mecanumDrive.driveCartesian(1.0, 0.0, 0.0);
            //         } else {
            //             mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
            //         }
            //     }
            //     if (isRingClose()) {
            //         navx.reset();
            //         if (navx.getAngle() < 5.0) { // Configure angle
            //             mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
            //         } else {
            //             mecanumDrive.stopMotor();
            //             /* Shoot */
            //             autonomousState++;
            //         }
            //     }
            //     break;
            // case 4:
            //     // Move forward until ultrasonic sensor detects something close, then turn and shoot
            //     if (!result.hasTargets() && !isRingClose()) {
            //         mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
            //     } else {
            //         double targetYaw = result.getBestTarget().getYaw();
            //         if (targetYaw > -1 && targetYaw < 1) {
            //             mecanumDrive.driveCartesian(1.0, 0.0, 0.0);
            //         } else {
            //             mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
            //         }
            //     }
            //     if (isRingClose()) {
            //         navx.reset();
            //         if (navx.getAngle() < 5.0) { // Configure angle
            //             mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
            //         } else {
            //             mecanumDrive.stopMotor();
            //             /* Shoot */
            //             autonomousState++;
            //         }
            //     }
            //     break;
        
   // }

    @Override
    public void teleopPeriodic() {
        // Mecanum drive control
        double x = joystick.getX();
        double y = joystick.getY();
        double rotation = joystick.getZ();

        // Apply deadband
        if (Math.abs(y) < 0.3) {
            y = 0;
        }
        if (Math.abs(x) < 0.3) {
            x = 0;
        }
        if (Math.abs(rotation) < 0.3) {
            rotation = 0;
        }
        // if (acc == true){
        //     driveSpeed*=10;
        // }

        mecanumDrive.driveCartesian(y * driveSpeed, -x * driveSpeed, -rotation * driveSpeed);

    //     // Intake control
    //     if (ps4Controller.getRawButton(5)) {
    //         intakeMotor.set(.5);
    //     } else if (ps4Controller.getRawButton(6)) {
    //         intakeMotor.set(-.5);
    //     } else {
    //         intakeMotor.set(0);
    //     }

    //     // Shooter control
    //     if (ps4Controller.getRawButton(7)) {
    //         shooterUpMotor.set(0.8);
    //         shooterDownMotor.set(0.8);
    //     } else if (ps4Controller.getRawButton(6)) {
    //         shooterUpMotor.set(-0.2);
    //         shooterDownMotor.set(-0.2);
    //     } else {
    //         shooterUpMotor.set(0);
    //         shooterDownMotor.set(0);
    //     }

    //     // Arm control
    //     if (ps4Controller.getRawButton(4)) {
    //         armMotor1.set(-0.2);
    //         // armMotor2.set(0.2);
    //     } else if (ps4Controller.getRawButton(2)) {
    //         armMotor1.set(0.2);
    //         // armMotor2.set(-0.2);
    //     } else {
    //         armMotor1.set(0);
    //         // armMotor2.set(0);
    //     }

            if (joystick.getRawButton(7)) { 
                armMotor1.set(0.5); 
               // armMotor2.set(0.2);

            } else if (joystick.getRawButton(8)) { 
                armMotor1.set(-0.5); 
               // armMotor2.set(-0.2);

            } else {
                armMotor1.set(0);
                //armMotor2.set(0);
            }
            if (joystick.getRawButtonPressed(11)){
                acc = !acc;
            }


    //     // // Climber control
    //     // if (joystick.getRawButton(2)) {
    //     //     climberMotor.set(0.5);
    //     // } else {
    //     //     climberMotor.set(0);
    //     // }

    //     // Reset arm encoder position
    //     // if (joystick.getRawButton(10)) {
    //     //     armEncoder.setPosition(0);
    //     // }

    //     // Dashboard output
    //     //33
    //     //-20.6 (2)

    //     SmartDashboard.putNumber("Arm Encoder Position", armEncoder.getPosition());
    //     // SmartDashboard.putNumber("Arm Encoder Position2", armEncoder2.getPosition());

    //    // SmartDashboard.putNumber("NavX Angle", navx.getAngle());
    //    // SmartDashboard.putNumber("Ultrasonic Distance (inches)", ultrasonicSensor.getRangeInches());
    }

    // Helper method to check if the ring is close
    private boolean isRingClose() {
        return ultrasonicSensor.getRangeInches() < 1;
    }
    public void armControllers(){
        if (joystick.getRawButtonPressed(1)){
          DEFAULT= true;
          FEED=SHOOT=SPEAKER = false;
        }
        if (joystick.getRawButtonPressed(2)){
          FEED= true;
          DEFAULT=SHOOT=SPEAKER = false;
        }
        if (joystick.getRawButtonPressed(3)){
          SHOOT= true;
          FEED=DEFAULT=SPEAKER = false;
        }
        if (joystick.getRawButtonPressed(4)){
          SPEAKER= true;
          FEED=SHOOT=DEFAULT = false;
        }
        ArmPID();
        
      }
      public void ArmPID(){
        if (DEFAULT){
          ArmOrientation(0.3,armEncoder.getPosition() , Constants.ShootConstants.DEFAULT);
          //ArmOrientation(0.3,armEncoder2.getPosition() ,Constants.ShootConstants.DEFAULT2,1);
        }
        else if (FEED){
          ArmOrientation(0.3,armEncoder.getPosition() , Constants.ShootConstants.FEED);//temp
          //ArmOrientation(0.3,armEncoder2.getPosition() , Constants.ShootConstants.FEED2,1);//temp
    
        }
        else if (SHOOT){
          ArmOrientation(0.3,armEncoder.getPosition() , Constants.ShootConstants.SHOOT);//temp
         // ArmOrientation(0.3,armEncoder2.getPosition() , Constants.ShootConstants.SHOOT2,1);//temp
    
    
        }
        else if (SPEAKER){
          ArmOrientation(0.3,armEncoder.getPosition() , Constants.ShootConstants.SPEAKER);//temp
         // ArmOrientation(0.3,armEncoder2.getPosition() , Constants.ShootConstants.SPEAKER2,1);//temp
    
        }
     
      }
      public void ArmOrientation(double power, double x, double r){
        armMotor1.set(power*((r-x)/r));
    //     if (type ==0){

    //     armMotor1.set(power*((r-x)/r));
    //   } else{
    //     armMotor2.set(power*((r-x)/r));

      
    }
}