package org.firstinspires.ftc.teamcode.CV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

public class IndicatorPositionPipeline extends OpenCvPipeline
{

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(30,40);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(120,40);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(210,40);
    static final int REGION_WIDTH = 50;
    static final int REGION_HEIGHT = 100;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    //TODO: CALLIBRATE REGIONS

    Point [][] points = {
            {
                    new Point(
                            REGION1_TOPLEFT_ANCHOR_POINT.x,
                            REGION1_TOPLEFT_ANCHOR_POINT.y),
                    new Point(
                            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT),
            },
            {
                    new Point(
                            REGION2_TOPLEFT_ANCHOR_POINT.x,
                            REGION2_TOPLEFT_ANCHOR_POINT.y),
                    new Point(
                            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT),
            },
            {
                    new Point(
                            REGION3_TOPLEFT_ANCHOR_POINT.x,
                            REGION3_TOPLEFT_ANCHOR_POINT.y),
                    new Point(
                            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT)
            }
    };


    Telemetry telemetry;

    Mat currentFrame;
    Mat[] regions = new Mat[3];
    String[] boxNames = {"LEFT", "MIDDLE", "RIGHT"};
    // Volatile since accessed by OpMode thread w/o synchronization
    int position = 0; //LEFT: 0, MIDDLE: 1, RIGHT: 2

    double[][] avgValues = new double[3][3]; //LEFT MIDDLE RIGHT
    double[] fitnesses = new double[3];
    double bestFitness = -1000;
    public IndicatorPositionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame)
    {

        currentFrame = firstFrame;
        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        regions[0] = currentFrame.submat(new Rect(points[0][0], points[0][1]));
        regions[1] = currentFrame.submat(new Rect(points[1][0], points[1][1]));
        regions[2] = currentFrame.submat(new Rect(points[2][0], points[2][1]));
    }

    public double fitness(double[] input) {
        return (input[1] - input[0] - input[2]) / Math.max(Math.max(input[0], input[1]), input[2]);
    }


    public Mat processFrame(Mat input) {
        boolean changed = false;
        bestFitness = -1000;
        for(int i=0; i<3; i++) {
            avgValues[i] = Core.mean(regions[i]).val;
            fitnesses[i] = fitness(avgValues[i]);
            if(fitnesses[i] > bestFitness) {
                changed = true;
                position = i;
                bestFitness = fitnesses[i];
            }
        }
        //telemetry.addData("Best Fitness", bestFitness);
        //telemetry.addData("Changed", changed);
        for(int i=0; i<3; i++)
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    points[i][0], // First point which defines the rectangle
                    points[i][1], // Second point which defines the rectangle
                    i == position ? GREEN: BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

        //for(int i=0; i<3; i++)
        //    telemetry.addData(boxNames[i] + " fitnesses", fitnesses[i]);

        //telemetry.addData("Best Position", boxNames[position]);
        //telemetry.update();
        return input;
    }

    public double[][] getAvgValues() {
        return avgValues;
    }

    public double[] getFitnesses(){ return fitnesses;}

    public int getAnalysis() {
        return position;
    }
}