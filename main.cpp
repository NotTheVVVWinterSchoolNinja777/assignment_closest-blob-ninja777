/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "closestBlob_IDL.h"

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   inPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   cropOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  targetPort;

    yarp::os::RpcClient rpc;

public:
    /********************************************************/

    Processing( const std::string &moduleName )
    {
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing()
    {

    };

    /********************************************************/
    bool open(){

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
        inPort.open("/"+ moduleName + "/image:i");
        outPort.open("/"+ moduleName + "/image:o");
        cropOutPort.open("/" + moduleName + "/crop:o");
        targetPort.open("/"+ moduleName + "/target:o");

        return true;
    }

    /********************************************************/
    void close()
    {
        inPort.close();
        outPort.close();
        targetPort.close();
        cropOutPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::interrupt();
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = outPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &cropOutImage  = cropOutPort.prepare();
        yarp::os::Bottle &outTargets = targetPort.prepare();

        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();

        outImage.resize(dispImage.width(), dispImage.height());
        cropOutImage.resize(dispImage.width(), dispImage.height());

        outImage.zero();
        cropOutImage.zero();

        cv::Mat inColour_cv = cv::cvarrToMat((IplImage *)inImage->getIplImage());  // prepare the image ports and targets
        cv::Mat disp = cv::cvarrToMat((IplImage *)dispImage.getIplImage());

        //FILL IN THE CODE

        // Apply image processing techniques on the disparity image to smooth things out
        yInfo()<<"Something useless";
        cv::Mat thres_disp = disp.clone();
        cv::GaussianBlur(disp, thres_disp, cv::Size(9, 9),1,1);


        // Apply some threshold on the image to remove background:
        // have a look at cv::threshold function
        //        yInfo<<disp.g


        cv::erode(thres_disp,thres_disp,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7)));
        cv::dilate(thres_disp,thres_disp,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));


        double min,  max;
        cv::minMaxLoc(thres_disp,&min, &max);

        yInfo()<<"Min :"<<min<<", Max :"<<max;
        cv::threshold(thres_disp, thres_disp,max-5,max+20,cv::ThresholdTypes::THRESH_BINARY_INV);
        // Find the max value and its position


        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thres_disp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        // Approximate contours to polygons + get bounding rects
        std::vector<std::vector<cv::Point>> contours_poly(contours.size());
        std::vector<cv::Rect> boundRect(contours.size());
        std::vector<std::vector<cv::Point>> hull(contours.size());
        cv::RNG rng(12345);
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

        for(int i = 0; i < contours.size(); i++)
        {
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
            cv::approxPolyDP(cv::Mat(hull[i]), contours_poly[i], 3, true);
            boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

        }

        bool isImageCropped = false;
        for( int i = 0; i< hull.size(); i++ )
        {
            cv::Mat out_image = inColour_cv.clone();

            cv::drawContours( disp, hull, i, color, 2, 8, hierarchy, 0, cv::Point() );
//            cv::rectangle( out_image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            if(!isImageCropped){

                cv::Point currentPoint = cv::Point((boundRect[i].x), (boundRect[i].y ));
                if(currentPoint.x == 0 || currentPoint.y ==0 ){
                    continue;
                }
                isImageCropped = true;
                yInfo()<<currentPoint.x<<" "<<currentPoint.y;
                cv::Rect roi(currentPoint.x, currentPoint.y, boundRect[i].width, boundRect[i].height);

                cv::Mat background(inColour_cv.size(), CV_8UC3, cv::Scalar(0,0,0));
                cv::Mat input_roi= inColour_cv(roi);
                yInfo()<<"1. Size of roi image :"<<input_roi.rows<<" x "<<input_roi.cols;
                cv::Mat foreground(input_roi.size(), CV_8UC3, cv::Scalar(0,0,0));
                yInfo()<<"2. Size of foreground image :"<<foreground.rows<<" x "<<foreground.cols;
                input_roi.copyTo(foreground, input_roi);
                yInfo()<<"3. Size of roi image :"<<input_roi.rows<<" x "<<input_roi.cols;
                yInfo()<<"4. Size of foreground image :"<<foreground.rows<<" x "<<foreground.cols;
                foreground.copyTo(background(cv::Rect(currentPoint.x, currentPoint.y, foreground.cols, foreground.rows)));
                yInfo()<<"5. Size of foreground image :"<<foreground.rows<<" x "<<foreground.cols;
                inColour_cv = background.clone();

                cv::Point center = (boundRect[i].br() + boundRect[i].tl())*0.5;
                cv::drawMarker(disp,center,cv::Scalar(255), cv::MARKER_TILTED_CROSS,2);
                outTargets.clear();
                yarp::os::Bottle  &outputBottle = outTargets.addList();
                outputBottle.addInt(boundRect[i].tl().x);
                outputBottle.addInt(boundRect[i].tl().y);
                outputBottle.addInt(boundRect[i].br().x);
                outputBottle.addInt(boundRect[i].br().y);

            }
       }

        //Find the contour of the closest objects with moments and mass center
        //

        //....

        // optional hint: you could use pointPolygonTest and the previous maxvalue location to compare with all contours found and get the actual brightest one

        //....

        // Use the result of pointPolygonTest or your own technique as the closest contour to:
        // 1 - draw it on the disparity image
        // 2 - create a cropped image containing the rgb roi
        // 3 - fill in a yarp bottle with the bounding box

        //be aware that the expected Bottle should be a list containing:
        // (tl.x tl.y br.x br.y)
        //where tl is top left and br - bottom right

        cvtColor(disp, disp, CV_GRAY2RGB);

        if (outTargets.size() >0 )
            targetPort.write();
        yInfo()<<"Converting disp to IPL";
        IplImage out = disp;
        outImage.resize(out.width, out.height);
        cvCopy( &out, (IplImage *) outImage.getIplImage());
        outPort.write();

        yInfo()<<"Converting inColor to IPL";
        IplImage crop = inColour_cv;
        cropOutImage.resize(crop.width, crop.height);
        cvCopy( &crop, (IplImage *) cropOutImage.getIplImage());
        cropOutPort.write();
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public closestBlob_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("closest-blob"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        processing = new Processing( moduleName );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}
