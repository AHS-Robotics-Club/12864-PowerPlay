package org.firstinspires.ftc.teamcode.auton.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PPConePipeline extends OpenCvPipeline{

    private Mat frame;

    Telemetry telemetery;

    //region Green
    private Scalar lowGreen; // Lower bound for green (HSV)
    private Scalar highGreen; // Higher bound for green (HSV)
    private Mat outputGreen; // What you see on the screen in EOCV; supposed to show only green pixels
    private Mat maskGreen; // Holds all the green pixels in webcam read
    private Point centroidGreen; // Center of green body
    //endregion

    //region Orange
    private Scalar lowOrange; // Lower bound for orange (HSV)
    private Scalar highOrange; // Higher bound for orange (HSV)
    private Mat outputOrange; // What you see on the screen in EOCV; supposed to show only orange pixels
    private Mat maskOrange; // Holds all the orange pixels in webcam read
    private Point centroidOrange; // Center of orange body
    //endregion

    //region Pink
    private Scalar lowPink; // Lower bound for pink (HSV)
    private Scalar highPink; // Higher bound for pink (HSV)
    private Mat outputPink; // What you see on the screen in EOCV; supposed to show only pink pixels
    private Mat maskPink; // Holds all the pink pixels in webcam read
    private Point centroidPink; // Center of pink body
    //endregion

    Mat submat;
    int color, pinkCount, greenCount, orangeCount;

    public PPConePipeline(){
        // A set of numbers representing the bounds of our desired color
        // We will compare the values the webcam reads with these values later
        lowGreen = new Scalar(35, 20, 20);
        highGreen = new Scalar(80, 255, 255);

        lowOrange = new Scalar(10, 50, 20);
        highOrange = new Scalar(30, 255, 255);

        lowPink = new Scalar(130, 50, 20);
        highPink = new Scalar(270, 255, 255);
    }


    @Override
    public void init(Mat firstFrame){
        submat = firstFrame.submat(firstFrame.rows()/2 - 5, firstFrame.rows()/2 + 5, firstFrame.cols()/2 - 5, firstFrame.cols()/2 + 5);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat original = new Mat();
        original = input;

        input = submat;

        input.convertTo(input, -1, 1, 50);
        original.convertTo(original, -1, 1  , 50);

        frame = new Mat();
        outputGreen = new Mat();
        outputOrange = new Mat();
        outputPink = new Mat();
        maskGreen = new Mat(outputGreen.rows(), outputGreen.cols(), CvType.CV_8UC4);
        maskOrange = new Mat(outputOrange.rows(), outputOrange.cols(), CvType.CV_8UC4);
        maskPink = new Mat(outputOrange.rows(), outputOrange.cols(), CvType.CV_8UC4);

        // The webcam reads image in RGB (Red-Green-Blue), but our bounds are in
        // HSV (Hue-Saturation-Value). We need to convert the image processing to
        // HSV.
        Imgproc.cvtColor(input, outputGreen, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, outputOrange, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, outputPink, Imgproc.COLOR_RGB2HSV);

        // Creates a mask of every single pixel of a certain color that's within bounds,
        // and then outputs on the screen.
        Core.inRange(outputGreen, lowGreen, highGreen, maskGreen); // Selects all green pixels
        Core.inRange(outputOrange, lowOrange, highOrange, maskOrange); // Selects all orange pixels
        Core.inRange(outputPink, lowPink, highPink, maskPink); // Selects all orange pixels
        // Puts the mask over the original image
        Core.bitwise_and(input, input, frame, maskGreen);
        Core.bitwise_and(input, input, frame, maskOrange);
        Core.bitwise_and(input, input, frame, maskPink);


        // Blurs the original image to smooth it out & get rid of the unwanted pixels
        Imgproc.GaussianBlur(maskGreen, maskGreen, new Size(11, 15), 0.0);
        Imgproc.GaussianBlur(maskOrange, maskOrange, new Size(11, 15), 0.0);
        Imgproc.GaussianBlur(maskPink, maskPink, new Size(11, 15), 0.0);

        //region Green Contours
        // Initialize Matrix and Array to find contours of object
        /*
         * A Mat of point gives the matrix for a singular point;
         * since we are trying to find all the contours, we need
         * an array list of MatPoints
         */
        List<MatOfPoint> contoursGreen = new ArrayList<>();
        Mat hierarchyGreen = new Mat(); // Going to store information about the topology of the frame (only for green objects)
        Imgproc.findContours(maskGreen, contoursGreen, hierarchyGreen, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //region Orange Contours
        // Initialize Matrix and Array to find contours of object
        /*
         * A Mat of point gives the matrix for a singular point;
         * since we are trying to find all the contours, we need
         * an array list of MatPoints
         */
        List<MatOfPoint> contoursOrange = new ArrayList<>();
        Mat hierarchyOrange = new Mat(); // Going to store information about the topology of the frame (only for orange objects)
        Imgproc.findContours(maskOrange, contoursOrange, hierarchyOrange, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //region Pink Contours
        // Initialize Matrix and Array to find contours of object
        /*
         * A Mat of point gives the matrix for a singular point;
         * since we are trying to find all the contours, we need
         * an array list of MatPoints
         */
        List<MatOfPoint> contoursPink = new ArrayList<>();
        Mat hierarchyPink = new Mat(); // Going to store information about the topology of the frame (only for Pink objects)
        Imgproc.findContours(maskPink, contoursPink, hierarchyPink, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Look at the biggest green contour and save it to the variable biggest
        if(hierarchyGreen.size().height > 0 && hierarchyGreen.size().width > 0){
            MatOfPoint biggestGreen = new MatOfPoint();
            // The last part of the for loop accesses whatevers on top of the screen topology
            for(int index = 0; index >= 0; index = (int)hierarchyGreen.get(0, index)[0]){
                Imgproc.drawContours(frame, contoursGreen, index, new Scalar(0, 0, 0));
                if(index == 0)
                    biggestGreen = contoursGreen.get(index);
                else if (contoursGreen.get(index).size().area() > contoursGreen.get(index - 1).size().area())
                    biggestGreen = contoursGreen.get(index);
            }

            // Creates a point and sets that point to the center of the largest contours
            Moments moments = Imgproc.moments(biggestGreen);
            centroidGreen = new Point();

            centroidGreen.x = moments.get_m10() / moments.get_m00();
            centroidGreen.y = moments.get_m01() / moments.get_m00();

            Rect rect = new Rect((int) centroidGreen.x, (int) centroidGreen.y, 10, 10);
            Imgproc.rectangle(frame, rect, new Scalar(255, 255, 255));
            Imgproc.putText(frame, "Green", centroidGreen, 1, 1, new Scalar(255, 255, 255));

            greenCount = biggestGreen.rows() * biggestGreen.cols();
        }
        //endregion

        if(hierarchyOrange.size().height > 0 && hierarchyOrange.size().width > 0){
            MatOfPoint biggestOrange = new MatOfPoint();
            // The last part of the for loop accesses whatevers on top of the screen topology
            for(int index = 0; index >= 0; index = (int)hierarchyOrange.get(0, index)[0]){
                Imgproc.drawContours(frame, contoursOrange, index, new Scalar(0, 0, 0));
                if(index == 0)
                    biggestOrange = contoursOrange.get(index);
                else if (contoursOrange.get(index).size().area() > contoursOrange.get(index - 1).size().area())
                    biggestOrange = contoursOrange.get(index);
            }

            // Creates a point and sets that point to the center of the largest contours
            Moments moments = Imgproc.moments(biggestOrange);
            centroidOrange = new Point();

            centroidOrange.x = moments.get_m10() / moments.get_m00();
            centroidOrange.y = moments.get_m01() / moments.get_m00();

            Rect rect = new Rect((int) centroidOrange.x, (int) centroidOrange.y, 10, 10);
            Imgproc.rectangle(frame, rect, new Scalar(255, 255, 255));
            Imgproc.putText(frame, "Orange", centroidOrange, 1, 1, new Scalar(255, 255, 255));

            orangeCount = biggestOrange.rows() * biggestOrange.cols();
        }

        // Creates a point and sets that point to the center of the largest contours
        if(hierarchyPink.size().height > 0 && hierarchyPink.size().width > 0){
            MatOfPoint biggestPink = new MatOfPoint();
            // The last part of the for loop accesses whatever's on top of the screen topology
            for(int index = 0; index >= 0; index = (int)hierarchyPink.get(0, index)[0]){
                Imgproc.drawContours(frame, contoursPink, index, new Scalar(100, 100, 100));
                if(index == 0)
                    biggestPink = contoursPink.get(index);
                else if (contoursPink.get(index).size().area() > contoursPink.get(index - 1).size().area())
                    biggestPink = contoursPink.get(index);
            }

            // Creates a point and sets that point to the center of the largest contours
            Moments moments = Imgproc.moments(biggestPink);
            centroidPink = new Point();

            centroidPink.x = moments.get_m10() / moments.get_m00();
            centroidPink.y = moments.get_m01() / moments.get_m00();

            Rect rect = new Rect((int) centroidPink.x, (int) centroidPink.y, 10, 10);
            Imgproc.rectangle(frame, rect, new Scalar(255, 255, 255));
            Imgproc.putText(frame, "Pink", centroidPink, 1, 1, new Scalar(255, 255, 255));

            pinkCount = biggestPink.rows() * biggestPink.cols();
        }
        //endregion

        telemetery.addData("Pink: ", pinkCount);
        telemetery.addData("Orange: ", orangeCount);
        telemetery.addData("Green: ", greenCount);


        if(greenCount == Math.max(greenCount, Math.max(pinkCount, orangeCount))){
            color = 1;
        }else if(orangeCount == Math.max(greenCount, Math.max(pinkCount, orangeCount))){
            color = 2;
        }else if(pinkCount == Math.max(greenCount, Math.max(pinkCount, orangeCount))){
            color = 3;
        }else{
            color = 0;
        }

        telemetery.update();

        maskGreen.release();
        maskOrange.release();
        maskPink.release();
        hierarchyGreen.release();
        hierarchyOrange.release();
        hierarchyPink.release();
        outputOrange.release();
        outputGreen.release();
        outputPink.release();

        Imgproc.putText(original, String.valueOf(color), new Point(15, 15), 1, 1, new Scalar(255, 255, 255));

        return original;
    }

    public int getColor(){
        return color;
    }
}