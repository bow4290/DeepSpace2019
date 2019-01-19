/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Spark;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

// import edu.wpi.first.wpilibj.TimedRobot;
/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  public DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  // private Joystick m_rightStick;
  Thread m_visionThread;
  NetworkTable table;
  private double totalAverage = 0.0;
  private UsbCamera camera;

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new Spark(2), new Spark(3));
    m_leftStick = new Joystick(0);
    table = NetworkTable.getTable("datatable");
    camera = CameraServer.getInstance().startAutomaticCapture();
    // m_rightStick = new Joystick(1);
    // JoystickButton visionButton = new JoystickButton(m_leftStick, 1);
    // visionButton.whenPressed(new StartVisionCommand());
    m_visionThread = new Thread(() -> {
      // // Get the UsbCamera from CameraServer
      // // Set the resolution
      camera.setResolution(640, 480);

      // // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getInstance().getVideo();
      // // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();
      GripPipeline pipeline = new GripPipeline();
      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.
      while (!Thread.interrupted()) {
        // Tell the CvSink to grab a frame from the camera and put it
        // // in the source mat. If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
          // // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // // skip the rest of the current iteration
          continue;
        }
        // Put a rectangle on the image
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
        // Give the output stream a new image to display
        pipeline.process(mat);
        ArrayList<MatOfPoint> blobPoints = pipeline.findContoursOutput();
        double totalAverage = 0;
        for (MatOfPoint matPoint : blobPoints) {
          double averageX = 0;
          Point[] currentPoints = matPoint.toArray();
          for (Point point : currentPoints) {
            averageX += point.x;
          }
          averageX = averageX / currentPoints.length;
          totalAverage += averageX;
        }
        totalAverage = totalAverage / blobPoints.size();
        table.putNumber("Total Average", totalAverage);

        outputStream.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(m_leftStick.getY(), m_leftStick.getX());
    // if (totalAverage< 145) {
    //   m_myRobot.arcadeDrive(0.2, 0.3);
    // } else if (totalAverage> 175){
    // m_myRobot.arcadeDrive(0.2, -0.3);
    // isCentered = true;
    }
  }
// }
