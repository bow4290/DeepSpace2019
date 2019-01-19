package frc.robot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import frc.robot.Robot;

import java.io.OutputStream;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 *
 */
public class StartVisionCommand extends Command {

    private VisionThread gripPipeline;
	private double centerX = 0.0;
	// private double tapeDistance = 0.0;
	private final Object imgLock = new Object();
	UsbCamera camera;
	private boolean isFinished = false;
    private boolean isCentered= false;
    private double totalAverage = 0.0;
    private Robot robot = new Robot();
    private NetworkTable table = NetworkTable.getTable("tables");
	
    // public DropGearWithCameraCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    // }


    
	public StartVisionCommand() {
		// Use requires() here to declare subsystem dependencies
		// require(Robot.);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        isCentered = false;
    	isFinished = false;
    	
	    camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(320, 240);
	    
    	gripPipeline = new VisionThread( camera, new GripPipeline(), pipeline -> {
            ArrayList<MatOfPoint> blobPoints = pipeline.findContoursOutput();
        double totalAverage = 0;
        synchronized(imgLock) {
            for(MatOfPoint matPoint : blobPoints)
        {
          double averageX = 0;
           Point[] currentPoints = matPoint.toArray();
           for(Point point : currentPoints)
           {
              averageX += point.x;
           }
           averageX = averageX / currentPoints.length;
           totalAverage += averageX;
        }
        totalAverage = totalAverage / blobPoints.size();
    }
        table.putNumber("Total Average",totalAverage);

        // OutputStream.putFrame(Mat);
            // if (!pipeline.findContoursOutput().isEmpty()) {
	        // 	if (pipeline.findContoursOutput().size() > 1) {
	        //         // Rect rectOne = Imgproc.boundingRect(pipeline.findContoursOutput().get(1));
	        //         // Rect rectTwo = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
			// 		synchronized (imgLock) {
			// 			// double centerRectOne = rectOne.x + (rectOne.width / 2);
			// 			// double centerRectTwo = rectTwo.x + (rectTwo.width / 2);
			// 			// tapeDistance = Math.abs(centerRectTwo - centerRectOne);
	        //             // centerX = (centerRectOne + centerRectTwo) / 2 - 10;
	        //         }
	        // 	} else {
	        //         // Rect rectOne = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
	        //         synchronized (imgLock) {
	        //             centerX = rectOne.x + (rectOne.width / 2);
	        //         }
	        // 	}
	        // }
        });
        gripPipeline.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        // double centerX;
    	// double tapeDistance;
    	synchronized (imgLock) {
    		// totalAverage = this.totalAverage;
    		// tapeDistance = this.tapeDistance;
    	}
    	
    	// double turn = centerX - (320.0 / 2)  * -1;  // BEFORE COMP REMOVE -1
    	
        table.putNumber("Total Average",totalAverage);
		// SmartDashboard.putNumber("Turn by", turn);
		// SmartDashboard.putNumber("Tape Distance", tapeDistance);

    	// BEFORE COMP ADD NEGATIVE BACK
    	if (totalAverage< 145) {
        	robot.m_myRobot.arcadeDrive(0.2, 0.3);
		} else if (totalAverage> 175){
			robot.m_myRobot.arcadeDrive(0.2, -0.3);
			// isCentered = true;
		} else {
			isFinished = true;
		}
    	
//    	if (isCentered) {
//    		if (tapeDistance > 0) {
//    			SmartDashboard.putNumber("Tape Distance", tapeDistance);
//
//    			Robot.driveTrain.driveTo(0.3, 0);
//    		} else {
//    			isFinished = true;
//    		}
//    	}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        camera = null;
        robot.m_myRobot.arcadeDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        end();

	}
}