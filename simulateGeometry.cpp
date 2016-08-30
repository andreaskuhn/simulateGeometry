
#include <iostream>

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>

#include "COLMAP/estimators/essential_matrix.h"
#include "Filereader.h"

struct Plane{
	Eigen::Vector3d center;
	Eigen::Vector3d normal;
};

Eigen::Matrix3d cameraA1, cameraA2;
Eigen::Matrix3d cameraR1, cameraR2;
Eigen::Vector3d cameraT1, cameraT2;
Eigen::Matrix3d E;

cv::Mat img1;
cv::Mat img2;
cv::Mat img3;
cv::Mat img1_orig;
cv::Mat img2_orig;

Plane plane1, plane2, plane3, plane4, plane5;

std::vector<Eigen::Vector2d> points2D_1;
std::vector<Eigen::Vector2d> points2D_2;
std::vector<Eigen::Vector2d> points2D_1_normalized;
std::vector<Eigen::Vector2d> points2D_2_normalized;

Filereader filereader;
int fileindex = 0;

double
getDepth(Eigen::Vector3d camPos, Eigen::Vector3d vec, Plane plane){
	double depth = -plane.normal.dot(camPos-plane.center) / plane.normal.dot(vec.normalized());
	if(depth<0)
		return std::numeric_limits<double>::max();
	return depth;
}

double
getMinDistFromPlanes(Eigen::Vector3d cameraT, Eigen::Vector3d p, int& i){
	double minDist=std::numeric_limits<double>::max();
	double dist1 = getDepth(cameraT, (p-cameraT), plane1);
	if(dist1 < minDist){
		minDist=dist1;
		i=1;
	}
	double dist2 = getDepth(cameraT, (p-cameraT), plane2);
	if(dist2 < minDist){
		minDist=dist2;
		i=2;
	}
	double dist3 = getDepth(cameraT, (p-cameraT), plane3);
	if(dist3 < minDist){
		minDist=dist3;
		i=3;
	}
	double dist4 = getDepth(cameraT, (p-cameraT), plane4);
	if(dist4 < minDist){
		minDist=dist4;
		i=4;
	}
	double dist5 = getDepth(cameraT, (p-cameraT), plane5);
	if(dist5 < minDist){
		minDist=dist5;
		i=5;
	}
	//std::cout << p[0] << " " << p[1] << " " << i << " " << minDist << std::endl;
	return minDist;
}

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONUP:
    {
    	std::cout << x << " " <<  y << "   ";
    	cv::circle(img1, cv::Point(x,y), 20, cv::Scalar(255, 255, 255), 1, 8);
    	img1.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
    	cv::imshow("IMG1", img1);
    	Eigen::Vector2d temp(x,y);
    	points2D_1.push_back(temp);
    	Eigen::Vector3d temp2(x,y,1);
    	temp2 = cameraA1.inverse()*temp2;
    	Eigen::Vector2d temp3(temp2[0], temp2[1]);
    	points2D_1_normalized.push_back(temp3);
    	std::cout << points2D_1_normalized[points2D_1_normalized.size()-1].transpose() << std::endl;
    }
    	break;
    case CV_EVENT_RBUTTONUP:
    {
    	img1_orig.copyTo(img1);
    	points2D_1.clear();
    	points2D_1_normalized.clear();
        std::cout << "Read from file: " << fileindex << std::endl;
        filereader.readParameters("../confs.txt", fileindex++, points2D_1);
        for(unsigned int i=0; i<points2D_1.size(); i++){
        	cv::circle(img1, cv::Point(points2D_1[i][0], points2D_1[i][1]), 20, cv::Scalar(255, 255, 255), 1, 8);
        	img1.at<cv::Vec3b>((unsigned char)points2D_1[i][1], (unsigned char)points2D_1[i][0]) = cv::Vec3b(255, 255, 255);
        	cv::imshow("IMG1", img1);
        	Eigen::Vector3d temp2(points2D_1[i][0],points2D_1[i][1],1);
        	temp2 = cameraA1.inverse()*temp2;
        	Eigen::Vector2d temp3(temp2[0], temp2[1]);
        	points2D_1_normalized.push_back(temp3);
        	std::cout << points2D_1_normalized[points2D_1_normalized.size()-1].transpose() << std::endl;
        }
    }
    	break;
    default:
    	break;
    }
}

void mouseHandler2(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONUP:
    	img1_orig.copyTo(img1);
    	img2_orig.copyTo(img2);
    	cv::circle(img1, cv::Point(x,y), 5, cv::Scalar(0, 255, 0), 1, 8);
    	img1.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
    	cv::imshow("IMG1", img1);

    	Eigen::Vector3d p1(x, y, 1);
    	Eigen::Vector3d p1_normalized = cameraA1.inverse()*p1;
    	//Eigen::Vector3d p1_normalized ((double)x/cameraA1(0,0), (double)y/cameraA1(0,0), 1);
    	std::cout << std::endl << x << " " <<  y << "   -> " << p1_normalized[0] << " " << p1_normalized[1] << std::endl;
    	//Eigen::Vector3d epiline = p1_normalized.transpose()*E;
    	Eigen::Vector3d epiline = E*p1_normalized;
    	std::cout << "epiline: " << epiline.transpose() << std::endl;

    	std::vector<Eigen::Vector3d> intersectpoints;
    	double y11 = -epiline[0]/epiline[1]*-0.5 - epiline[2]/epiline[1];
    	if(y11>=-0.5 && y11<0.5){
    		Eigen::Vector3d temp(-0.5, y11, 1);
    		intersectpoints.push_back(temp);
    	}
    	double x11 = -epiline[1]/epiline[0]*-0.5 - epiline[2]/epiline[0];
    	if(x11>=-0.5 && x11<0.5){
    		Eigen::Vector3d temp(x11, -0.5, 1);
    		intersectpoints.push_back(temp);
    	}
    	if(intersectpoints.size()<2){
        	double y11 = -epiline[0]/epiline[1]*0.5 - epiline[2]/epiline[1];
        	if(y11>=-0.5 && y11<0.5){
        		Eigen::Vector3d temp(0.5, y11, 1);
        		intersectpoints.push_back(temp);
        	}
    	}
    	if(intersectpoints.size()<2){
        	double x11 = -epiline[1]/epiline[0]*0.5 - epiline[2]/epiline[0];
        	if(x11>=-0.5 && x11<0.5){
        		Eigen::Vector3d temp(x11, 0.5, 1);
        		intersectpoints.push_back(temp);
        	}
    	}
    	if(intersectpoints.size()>=2){
    		intersectpoints[0] = cameraA2*intersectpoints[0];
    		intersectpoints[1] = cameraA2*intersectpoints[1];
    		std::cout << intersectpoints[0].transpose() << std::endl;
    		std::cout << intersectpoints[1].transpose() << std::endl;
			cv::line(img2, cv::Point(intersectpoints[0][0], intersectpoints[0][1]),
						   cv::Point(intersectpoints[1][0], intersectpoints[1][1]),
						   cv::Scalar(255, 255, 255), 3);
    	}

    	p1 = cameraA1.inverse()*p1;
    	Eigen::Vector3d p1_o = p1;
    	int i;
    	double d2 = getMinDistFromPlanes(cameraT1, p1, i);
    	std::cout << "mindistfromplane: " << d2 << std::endl;
    	p1*=d2;
    	p1 = cameraR2.inverse()*(p1 - cameraT2);
    	p1 = p1/p1[2];
    	std::cout << "Check: " << p1.transpose()*E*p1_o << std::endl;
    	std::cout << p1.transpose() << std::endl;
    	std::cout << p1_o.transpose() << std::endl;
    	double dist = fabs(double(p1[0]*epiline[0] + p1[1]*epiline[1] + epiline[2]))
    			/(sqrt(double(epiline[0]*epiline[0]+epiline[1]*epiline[1])));
    	std::cout << "dist: " << dist*cameraA2(0,0) << std::endl;
    	p1 = cameraA2*p1;
    	cv::circle(img2, cv::Point(p1[0],p1[1]), 20, cv::Scalar(255, 0, 0), 1, 8);
    	cv::imshow("IMG2", img2);
    }
}

void
getPointsFromPlane(Plane plane, double distance, std::vector<Eigen::Vector3d>& points3D){
	double dist = distance;
	for(int i=0; i<2; i++, dist+=distance){
		points3D[i]=Eigen::Vector3d(dist,dist,0);
		points3D[i+1]=Eigen::Vector3d(dist,-dist,0);
		points3D[i+2]=Eigen::Vector3d(-dist,dist,0);
		points3D[i+3]=Eigen::Vector3d(-dist,-dist,0);
	}
	Eigen::Vector3d unit;
	unit << 0 , 0, 1;
	Eigen::Quaternion<double> q = Eigen::Quaterniond().setFromTwoVectors(plane.normal, unit);
	Eigen::Matrix3d rot = q.toRotationMatrix();
	for(unsigned int i=0; i<8; i++){
		points3D[i]=rot*points3D[i];
		points3D[i]+=plane.center;
		std::cout << points3D[i].transpose() << std::endl;
	}
}

void
projectPointsToImage(std::vector<Eigen::Vector2d>& points2D_1,
		             std::vector<Eigen::Vector2d>& points2D_2){
	points2D_2.resize(points2D_1.size());
	points2D_2_normalized.resize(points2D_1.size());
	for(unsigned int i=0; i<points2D_1.size(); i++){
		Eigen::Vector3d p(points2D_1[i][0], points2D_1[i][1], 1);
		p=cameraA1.inverse()*p;
		int dummy=0;
		p*=getMinDistFromPlanes(cameraT1, p, dummy);
		//Eigen::Vector3d temp = cameraR2*p + cameraT2;
		Eigen::Vector3d temp = cameraR2.inverse()*(p - cameraT2);
		temp = temp/temp[2];
		points2D_2_normalized[i][0] = temp[0];
		points2D_2_normalized[i][1] = temp[1];
		temp = cameraA2*temp;
    	cv::circle(img2, cv::Point(temp[0],temp[1]), 20, cv::Scalar(255, 255, 255), 1, 8);
    	img2.at<cv::Vec3b>((unsigned char)temp[1], (unsigned char)temp[1]) = cv::Vec3b(255, 255, 255);
    	cv::imshow("IMG2", img2);
		points2D_2[i][0] = temp[0];
		points2D_2[i][1] = temp[1];
		std::cout   << points2D_1[i].transpose()
		  << " -> " << points2D_2[i].transpose()
		  << "    " << points2D_2_normalized[i].transpose() << std::endl;
	}
}

void
colorImage(cv::Mat& img, int x, int y, int i){
	switch(i){
	case 1:
		img.at<cv::Vec3b>(y,x)[0] = 0;
		img.at<cv::Vec3b>(y,x)[1] = 0;
		img.at<cv::Vec3b>(y,x)[2] = 0;
		break;
	case 2:
		img.at<cv::Vec3b>(y,x)[0] = 255;
		img.at<cv::Vec3b>(y,x)[1] = 0;
		img.at<cv::Vec3b>(y,x)[2] = 0;
		break;
	case 3:
		img.at<cv::Vec3b>(y,x)[0] = 0;
		img.at<cv::Vec3b>(y,x)[1] = 127;
		img.at<cv::Vec3b>(y,x)[2] = 0;
		break;
	case 4:
		img.at<cv::Vec3b>(y,x)[0] = 0;
		img.at<cv::Vec3b>(y,x)[1] = 0;
		img.at<cv::Vec3b>(y,x)[2] = 255;
		break;
	case 5:
		img.at<cv::Vec3b>(y,x)[0] = 127;
		img.at<cv::Vec3b>(y,x)[1] = 127;
		img.at<cv::Vec3b>(y,x)[2] = 0;
		break;
	default:
		img.at<uchar>(y,x,0) = 255;
		img.at<uchar>(y,x,1) = 255;
		img.at<uchar>(y,x,2) = 255;
		break;
	}
}

Eigen::Matrix3d
getXRot(double angle){
	Eigen::Matrix3d rotX;
	rotX << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
	return rotX;
}

Eigen::Matrix3d
getYRot(double angle){
	Eigen::Matrix3d rotY;
	rotY << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle);
	return rotY;
}

Eigen::Matrix3d
getZRot(double angle){
	Eigen::Matrix3d rotZ;
	rotZ << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
	return rotZ;
}

int main(int argc, char **argv){

	int eindex = atof(argv[4]);
	int imgsize = 1000;
	img1 = cv::Mat(imgsize, imgsize, CV_8UC3);
	img2 = cv::Mat(imgsize, imgsize, CV_8UC3);
	img3 = cv::Mat(imgsize, imgsize, CV_8UC1);

	cameraA1 << imgsize, 0, imgsize/2, 0, imgsize, imgsize/2, 0, 0, 1;
	cameraA2 << imgsize, 0, imgsize/2, 0, imgsize, imgsize/2, 0, 0, 1;
	cameraR1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	cameraR2 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	cameraT1 << 0, 0, 0;
	cameraT2 << 1.0, 0, 0;
	cameraR2=getXRot((atof(argv[1])/180.0)*M_PI)*cameraR2;
	cameraR2=getYRot((atof(argv[2])/180.0)*M_PI)*cameraR2;
	cameraR2=getZRot((atof(argv[3])/180.0)*M_PI)*cameraR2;

	double distZ = 60; //z-dist
	double dist2 = distZ/5;
	double angle1 = (10.0/180.0)*M_PI; //blue
	double angle2 = (90.0/180.0)*M_PI; //green
	double angle3 = (20.0/180.0)*M_PI;
	double angle4 = (15.0/180.0)*M_PI;

	plane1.center << cameraT2[0]/2+2.0, 0, cameraT2[0]*distZ;
	plane1.normal << 0, 0, -1;

	plane2.center = plane1.center;
	plane2.center[0]-=dist2;
	plane2.center[1]-=dist2;
	plane2.normal << 0, 0, -1;
	plane2.normal = getYRot(-angle1)*plane2.normal;

	plane3.center = plane1.center;
	plane3.center[0]-=dist2;
	plane3.center[1]+=dist2;
	plane3.normal << 0, 0, -1;
	plane3.normal = getXRot(-angle2)*plane3.normal;

	plane4.center = plane1.center;
	plane4.center[0]+=dist2;
	plane4.center[1]-=dist2;
	plane4.normal << 0, 0, -1;
	plane4.normal = getXRot(angle3)*plane4.normal;

	plane5.center = plane1.center;
	plane5.center[0]+=dist2;
	plane5.center[1]+=dist2;
	plane5.normal << 0, 0, -1;
	plane5.normal = getYRot(angle4)*plane5.normal;

	//images sim
	for(int x=0; x<imgsize; x++){
		for(int y=0; y<imgsize; y++){
			Eigen::Vector3d p(x, y, 1);
			p=cameraA1.inverse()*p;
			int i=0;
			getMinDistFromPlanes(cameraT1, p, i);
			colorImage(img1, x, y, i);

			Eigen::Vector3d p2(x, y, 1);
			p2=cameraA2.inverse()*p2;
			p2 = cameraR2*p2 + cameraT2;
			int i2=0;
			getMinDistFromPlanes(cameraT2, p2, i2);
			colorImage(img2, x, y, i2);
		}
	}

	//cv::imwrite("test.jpg", img1);
	//std::cout << "Going to estimate points from plane" << std::endl;
	//std::vector<Eigen::Vector3d> points3D(8);
	//getPointsFromPlane(plane1, 0.01, points3D);


	//Klick points
	img1.copyTo(img1_orig);
	cv::namedWindow("IMG1", cv::WINDOW_NORMAL);
	int mouseParam= CV_EVENT_FLAG_LBUTTON;
	cvSetMouseCallback("IMG1", mouseHandler, &mouseParam);
	cv::imshow("IMG1", img1);
	cv::moveWindow("IMG1", 10, 50);
	cv::namedWindow("IMG2", cv::WINDOW_NORMAL);
	cv::imshow("IMG2", img2);
	cv::moveWindow("IMG2", 10, 400);
	cv::waitKey(0);


	//Project to 2nd image
	projectPointsToImage(points2D_1, points2D_2);

	//std::cout << "Essent" << std::endl;
	std::vector<cv::Point2d> points2D_1_cv(points2D_1.size());
	std::vector<cv::Point2d> points2D_2_cv(points2D_1.size());
	for(unsigned int i=0; i<points2D_1.size(); i++){
		points2D_1_cv[i].x=points2D_1_normalized[i][0];
		points2D_1_cv[i].y=points2D_1_normalized[i][1];
		points2D_2_cv[i].x=points2D_2_normalized[i][0];
		points2D_2_cv[i].y=points2D_2_normalized[i][1];
		/*double max = 0.0;
	    double f = (double)rand() / RAND_MAX;
	    double rand = f * max*2;
	    rand -= max;
	    std::cout << rand << std::endl;
	    points2D_2_cv[i].x+=rand;
		points2D_2_cv[i].y+=rand;*/
	}

	cv::Mat E_cv;
	if(points2D_1.size()>4){

		cv::Mat mask;
		E_cv = cv::findEssentialMat(points2D_1_cv, points2D_2_cv, 1.0, cv::Point2d(0, 0), cv::RANSAC, 0.999, 1.0, mask);
		std ::cout << E_cv.dims << std::endl;
		std ::cout << E_cv.type() << std::endl;
		//cv::Mat E_cv = findFundamentalMat(points2D_1_cv, points2D_2_cv);
		std::cout << E_cv << std::endl;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				E(j,i)=E_cv.at<cv::Vec3d>(j, i)[eindex];
			}
		}


		/*colmap::EssentialMatrixFivePointEstimator emfpe;
		//colmap::EssentialMatrixEightPointEstimator emfpe;
		E = emfpe.Estimate(points2D_1_normalized, points2D_2_normalized)[eindex];
		E_cv = cv::Mat(3, 3, CV_32FC1);
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				E_cv.at<double>(i,j)=E(j,j);
			}
		} */

		//std::cout << mask << std::endl;
		//std::cout << std::endl;
	}
	else{
		std::cout << "Essential Matrix from R/T" << std::endl;
		E_cv = cv::Mat(3, 3, CV_32FC1);

		Eigen::Matrix3d R=cameraR2.transpose();
		//Eigen::Vector3d transl=-R*cameraT2;
		//Eigen::Matrix3d R = cameraR2*cameraR1.transpose();
		//Eigen::Vector3d transl = cameraT2 - R*cameraT1;
		//Eigen::Matrix3d R = cameraR2;
		Eigen::Vector3d transl = cameraT2;
		Eigen::Matrix3d S;
		S(0,0)=0.0; S(0,1)=-transl[2]; S(0,2)=transl[1];
		S(1,0)=transl[2]; S(1,1)=0.0; S(1,2)=-transl[0];
		S(2,0)=-transl[1]; S(2,1)=transl[0]; S(2,2)=0;
		//E=cameraA1.inverse().transpose() * S * R * cameraA2.inverse();
		//E = S * R;
		E = R * S;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				E_cv.at<double>(i,j)=E(j,i);
			}
		}
	}
	//std::cout << transl << std::endl;
	std::cout << "Essentiel Matrix: " << E << std::endl;


	img1.copyTo(img1_orig);
	img2.copyTo(img2_orig);
	mouseParam= CV_EVENT_FLAG_LBUTTON;
	cvSetMouseCallback("IMG1", mouseHandler2, &mouseParam);
	cv::imshow("IMG2", img2);
	cv::waitKey(0);

	std::cout << "Error estimation" << std::endl;
	cv::Mat imgT = cv::Mat(imgsize, imgsize, CV_32FC1);
	double maxDist = 0;
	for(int x=0; x<imgsize; x++){
		for(int y=0; y<imgsize; y++){
			//cv::Vec3b col = img1.at<cv::Vec3b>(y,x);
			//if(!(col[0]==0 && col[1]==0 && col[2]==0) && !(col[0]==0 && col[1]==127 && col[2]==0))
			//	continue;
			Eigen::Vector3d p(x, y, 1);
			p = cameraA1.inverse()*p;
	    	Eigen::Vector3d p1_normalized = p;
	    	Eigen::Vector3d epiline = E*p1_normalized;
	    	int i;
	    	double d2 = getMinDistFromPlanes(cameraT1, p, i);
	    	p *= d2;
	    	p = cameraR2.inverse()*(p - cameraT2);
	    	p = p/p[2];
	    	double dist = fabs(double(p[0]*epiline[0] + p[1]*epiline[1] + epiline[2]))
	    			/ (sqrt(double(epiline[0]*epiline[0]+epiline[1]*epiline[1])));
	    	dist*=cameraA2(0,0);
	    	//std::cout << "dist: " << dist << std::endl;
	    	if(dist>maxDist)
	    		maxDist=dist;
	    	imgT.at<float>(y,x) = dist;
		}
	}

	for(int x=0; x<imgsize; x++){
		for(int y=0; y<imgsize; y++){
//			cv::Vec3b col = img1.at<cv::Vec3b>(y,x);
//			if(!(col[0]==0 && col[1]==0 && col[2]==0) && !(col[0]==0 && col[1]==127 && col[2]==0))
//				continue;

			float dist = imgT.at<float>(y,x);
			double range = 10.0;
			//double range = maxDist - 5;
			int val = 255-(255*dist/range);
			if(val<0)
				val=0;
			if(val>255)
				val=255;
			img3.at<uchar>(y,x) = val;
		}
	}
	std::cout << "maxDist " << maxDist << std::endl;

	if(points2D_1.size()>5){
		cv::Mat rrot1;
		cv::Mat rrot2;
		cv::Mat ttransl;
		cv::decomposeEssentialMat(E_cv, rrot1, rrot2, ttransl);
		Eigen::Matrix3d R = cameraR2*cameraR1.transpose();
		Eigen::Vector3d transl = cameraT2 - R*cameraT1;
		std::cout << std::endl;
		std::cout << transl << std::endl;
		std::cout << ttransl << std::endl;
		std::cout << std::endl;
		std::cout << R << std::endl;
		std::cout << rrot1 << std::endl;
		std::cout << rrot2 << std::endl;
	}

	cv::namedWindow("IMG3", cv::WINDOW_NORMAL);
	cv::imshow("IMG3", img3);
	cv::waitKey(0);

	//cv::imwrite("test.jpg", img2);
	return 0;
}


