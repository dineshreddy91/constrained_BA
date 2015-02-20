/* ceres_functions.h
 *
 * Copyright (c)2007 Visesh Chari <visesh [at] research.iiit.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * Changelog:
 * Thu 24 May 2007 11:01:40 PM IST Created
 */

#ifndef CERES_FUNCTIONS_H
#define CERES_FUNCTIONS_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;
#define EPS T(0.00001)

extern double fx, fy, px, py, w3Dv2D;
extern double *object_half_size, *object_weight;
 
struct AlignmentErrorTriangulate 
{
	AlignmentErrorTriangulate(double* observed_in, double* camera_extrinsic_in): observed(observed_in), camera_extrinsic(camera_extrinsic_in) {}

	template <typename T>
		bool operator()(const T* const point, T* residuals) const 
		{
			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];

			ceres::AngleAxisRotatePoint((T*)(camera_extrinsic), point, p);

			// camera_extrinsic[3,4,5] are the translation.
			p[0] += T(camera_extrinsic[3]);
			p[1] += T(camera_extrinsic[4]);
			p[2] += T(camera_extrinsic[5]);

			// let p[2] ~= 0
			if (T(0.0)<=p[2]){
				if(p[2]<EPS){
					p[2] = EPS;
				}
			}else{
				if (p[2]>-EPS){
					p[2] = -EPS;
				}
			}

			// project it
			p[0] = T(fx) * p[0] / p[2] + T(px);
			p[1] = T(fy) * p[1] / p[2] + T(py);

			// reprojection error
			residuals[0] = (p[0] - T(observed[0]));
			residuals[1] = (p[1] - T(observed[1]));     

			/*
			   std::cout<<"p[0]="<<p[0]<<std::endl;
			   std::cout<<"p[1]="<<p[1]<<std::endl;
			   std::cout<<"observed[0]="<<observed[0]<<std::endl;
			   std::cout<<"observed[1]="<<observed[1]<<std::endl;
			   std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			   std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
			   std::cout<<"--------------------------"<<std::endl;
			   */
			return true;
		}

	double* observed;
	double* camera_extrinsic;
};


struct AlignmentError2D 
{
	AlignmentError2D(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const point,
				T* residuals) const {

			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];

			ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
			/*
			   T x = camera_extrinsic[0];
			   T y = camera_extrinsic[1];
			   T z = camera_extrinsic[2];
			   T x2 = x*x;
			   T y2 = y*y;
			   T z2 = z*z;    
			   T w2 = T(1.0) - x2 - y2 - z2;
			   T w  = sqrt(w2);

			   p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
			   p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
			   p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
			   */

			// camera_extrinsic[3,4,5] are the translation.
			p[0] += camera_extrinsic[3];
			p[1] += camera_extrinsic[4];
			p[2] += camera_extrinsic[5];

			// let p[2] ~= 0
			if (T(0.0)<=p[2]){
				if(p[2]<EPS){
					p[2] = EPS;
				}
			}else{
				if (p[2]>-EPS){
					p[2] = -EPS;
				}
			}

			// project it
			p[0] = T(fx) * p[0] / p[2] + T(px);
			p[1] = T(fy) * p[1] / p[2] + T(py);

			// reprojection error
			residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
			residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

			/*
			   if (exe_time<10000){
			   exe_time++;
			   std::cout<<"p[0]="<<p[0]<<std::endl;
			   std::cout<<"p[1]="<<p[1]<<std::endl;
			   std::cout<<"observed[0]="<<observed[0]<<std::endl;
			   std::cout<<"observed[1]="<<observed[1]<<std::endl;
			   std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			   std::cout<<"residuals[1]="<<residuals[1]<<std::endl;        
			   std::cout<<"--------------------------"<<std::endl;
			   }
			   */

			return true;
		}

	double* observed;

};


struct AlignmentError3D 
{
	AlignmentError3D(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const point,
				T* residuals) const {

			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];

			ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
			/*
			   T x = camera_extrinsic[0];
			   T y = camera_extrinsic[1];
			   T z = camera_extrinsic[2];
			   T x2 = x*x;
			   T y2 = y*y;
			   T z2 = z*z;    
			   T w2 = T(1.0) - x2 - y2 - z2;
			   T w  = sqrt(w2);

			   p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
			   p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
			   p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
			   */

			// camera_extrinsic[3,4,5] are the translation.
			p[0] += camera_extrinsic[3];
			p[1] += camera_extrinsic[4];
			p[2] += camera_extrinsic[5];

			// The error is the difference between the predicted and observed position.
			residuals[0] = T(observed[5])*(p[0] - T(observed[2]));
			residuals[1] = T(observed[5])*(p[1] - T(observed[3]));
			residuals[2] = T(observed[5])*(p[2] - T(observed[4]));


			/*    
				  if (exe_time<10){
				  exe_time ++;
				  std::cout<<"fx="<<fx<<std::endl;
				  std::cout<<"fy="<<fy<<std::endl;
				  std::cout<<"px="<<px<<std::endl;
				  std::cout<<"py="<<py<<std::endl;
				  std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;        
				  std::cout<<"p[0]="<<p[0]<<std::endl;
				  std::cout<<"p[1]="<<p[1]<<std::endl;
				  std::cout<<"p[2]="<<p[2]<<std::endl;
				  std::cout<<"observed[0]="<<observed[0]<<std::endl;
				  std::cout<<"observed[1]="<<observed[1]<<std::endl;
				  std::cout<<"observed[2]="<<observed[2]<<std::endl;
				  std::cout<<"observed[3]="<<observed[3]<<std::endl;
				  std::cout<<"observed[4]="<<observed[4]<<std::endl;
				  std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
				  std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
				  std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
				  std::cout<<"--------------------------"<<std::endl;
				  }
				  */

			float n[4]= {0,1,0,0};
			T temp = (camera_extrinsic[3]* T(n[0]))+(camera_extrinsic[4] * T(n[1]))+(camera_extrinsic[5] * T(n[2]));



			return true;
		}
	double* observed;
};


template <class T>
T bandFunction(T x, T halfWin)
{
	return T(1.0)/(T(1.0)+exp(T(-100.0)*(x-halfWin))) + T(1.0)/(T(1.0)+exp(T(-100.0)*(-x-halfWin)));
}

template <class T>
T hingeLoss(T x, T halfWin)
{
	//return max(T(0), T(-halfWin)-x) + max(T(0), T(-halfWin)+x);
	if (x< -halfWin){
		//return -halfWin - x;
		return x + halfWin;
	}else if (x<halfWin){
		return 0;
	}else{
		return x-halfWin;
	}
}


struct AlignmentErrorBox 
{
	AlignmentErrorBox(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const world2object,
				T* residuals) const {

			T p[3];
			T q[3];  
			T o[3];  
			T r[3];

			p[0] = T(observed[2]) - camera_extrinsic[3];
			p[1] = T(observed[3]) - camera_extrinsic[4];
			p[2] = T(observed[4]) - camera_extrinsic[5];

			r[0] = - camera_extrinsic[0];
			r[1] = - camera_extrinsic[1];
			r[2] = - camera_extrinsic[2];

			ceres::AngleAxisRotatePoint(r, p, o);

			ceres::AngleAxisRotatePoint(world2object, o, q);

			q[0] += world2object[3];
			q[1] += world2object[4];
			q[2] += world2object[5];

			// quadratic function
			//residuals[0] = q[0];
			//residuals[1] = q[1];
			//residuals[2] = q[2];    

			// hinge loss
			double* szPtr = object_half_size + 3 * int(observed[5]);
			T oW = T(object_weight[int(observed[5])]);

			//std::cout<<"oW="<<oW<<std::endl;

			//residuals[0] = hingeLoss<T>(q[0], T(szPtr[0]));
			//residuals[1] = hingeLoss<T>(q[1], T(szPtr[1]));
			//residuals[2] = hingeLoss<T>(q[2], T(szPtr[2])); 

			if (q[0] < T(-szPtr[0])){
				residuals[0] = oW * (q[0] + T(szPtr[0]));
			}else if (q[0] < T(szPtr[0])){
				residuals[0] = T(0.0);
			}else{
				residuals[0] = oW * (q[0] - T(szPtr[0]));
			}

			if (q[1] < T(-szPtr[1])){
				residuals[1] = oW * (q[1] + T(szPtr[1]));
			}else if (q[1] < T(szPtr[1])){
				residuals[1] = T(0.0);
			}else{
				residuals[1] = oW * (q[1] - T(szPtr[1]));
			}

			if (q[2] < T(-szPtr[2])){
				residuals[2] = oW * (q[2] + T(szPtr[2]));
			}else if (q[2] < T(szPtr[2])){
				residuals[2] = T(0.0);
			}else{
				residuals[2] = oW * (q[2] - T(szPtr[2]));
			}    

			//double* szPtr = object_half_size + 3 * int(observed[5]);
			//residuals[0] = bandFunction<T>(q[0], T(szPtr[0]));
			//residuals[1] = bandFunction<T>(q[1], T(szPtr[1]));
			//residuals[2] = bandFunction<T>(q[2], T(szPtr[2]));

			//std::cout<<"szPtr[0]="<<szPtr[0]<<std::endl;
			//std::cout<<"szPtr[1]="<<szPtr[1]<<std::endl;
			//std::cout<<"szPtr[2]="<<szPtr[2]<<std::endl;
			//std::cout<<"q[0]="<<q[0]<<std::endl;
			//std::cout<<"q[1]="<<q[1]<<std::endl;
			//std::cout<<"q[2]="<<q[2]<<std::endl;

			//std::cout<<"bandFunction<T>(q[0], T(szPtr[0]))="<<bandFunction<T>(q[0], T(szPtr[0]))<<std::endl;
			//std::cout<<"bandFunction<T>(q[0], T(szPtr[0]))="<<bandFunction<T>(q[1], T(szPtr[1]))<<std::endl;
			//std::cout<<"bandFunction<T>(q[0], T(szPtr[0]))="<<bandFunction<T>(q[2], T(szPtr[2]))<<std::endl;




			return true;
		}
	double* observed;
};

struct AlignmentErrorAxisManhattanBox 
{
	AlignmentErrorAxisManhattanBox(double* observed_in): observed(observed_in) {}
	template <typename T>
		bool operator()(const T* const camera_extrinsic,const T* const world2object,T* residuals) const {
			T p[3];
			T q[3];  
			T o[3];  
			T r[3];
			p[0] = T(observed[2]) - camera_extrinsic[3];
			p[1] = T(observed[3]) - camera_extrinsic[4];
			p[2] = T(observed[4]) - camera_extrinsic[5];
			r[0] = - camera_extrinsic[0];
			r[1] = - camera_extrinsic[1];
			r[2] = - camera_extrinsic[2];
			ceres::AngleAxisRotatePoint(r, p, o);

			q[0] = o[0] + world2object[0];
			q[1] = o[1] + world2object[1];
			q[2] = o[2] + world2object[2];

			double* szPtr = object_half_size + 3 * int(observed[5]);
			T oW = T(object_weight[int(observed[5])]);
			if (q[0] < T(-szPtr[0])){
				residuals[0] = oW * (q[0] + T(szPtr[0]));
			}else if (q[0] < T(szPtr[0])){
				residuals[0] = T(0.0);
			}else{
				residuals[0] = oW * (q[0] - T(szPtr[0]));
			}
			if (q[1] < T(-szPtr[1])){
				residuals[1] = oW * (q[1] + T(szPtr[1]));
			}else if (q[1] < T(szPtr[1])){
				residuals[1] = T(0.0);
			}else{
				residuals[1] = oW * (q[1] - T(szPtr[1]));
			}
			if (q[2] < T(-szPtr[2])){
				residuals[2] = oW * (q[2] + T(szPtr[2]));
			}else if (q[2] < T(szPtr[2])){
				residuals[2] = T(0.0);
			}else{
				residuals[2] = oW * (q[2] - T(szPtr[2]));
			}    
			return true;
		}
	double* observed;
};


struct AlignmentErrorAxisBox 
{
	AlignmentErrorAxisBox(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const world2object,
				T* residuals) const {

			T p[3];
			T q[3];  
			T o[3];  
			T r[3];

			p[0] = T(observed[2]) - camera_extrinsic[3];
			p[1] = T(observed[3]) - camera_extrinsic[4];
			p[2] = T(observed[4]) - camera_extrinsic[5];

			r[0] = - camera_extrinsic[0];
			r[1] = - camera_extrinsic[1];
			r[2] = - camera_extrinsic[2];

			ceres::AngleAxisRotatePoint(r, p, o);


			T cosTheta = cos(world2object[0]);
			T sinTheta = sin(world2object[0]);


			q[0] = cosTheta * o[0] - sinTheta * o[2] + world2object[1];
			q[1] = o[1]                              + world2object[2];
			q[2] = sinTheta * o[0] + cosTheta * o[2] + world2object[3];


			// hinge loss
			double* szPtr = object_half_size + 3 * int(observed[5]);
			T oW = T(object_weight[int(observed[5])]);
			//std::cout<<"oW="<<oW<<std::endl;

			//residuals[0] = hingeLoss<T>(q[0], T(szPtr[0]));
			//residuals[1] = hingeLoss<T>(q[1], T(szPtr[1]));
			//residuals[2] = hingeLoss<T>(q[2], T(szPtr[2])); 

			if (q[0] < T(-szPtr[0])){
				residuals[0] = oW * (q[0] + T(szPtr[0]));
			}else if (q[0] < T(szPtr[0])){
				residuals[0] = T(0.0);
			}else{
				residuals[0] = oW * (q[0] - T(szPtr[0]));
			}

			if (q[1] < T(-szPtr[1])){
				residuals[1] = oW * (q[1] + T(szPtr[1]));
			}else if (q[1] < T(szPtr[1])){
				residuals[1] = T(0.0);
			}else{
				residuals[1] = oW * (q[1] - T(szPtr[1]));
			}

			if (q[2] < T(-szPtr[2])){
				residuals[2] = oW * (q[2] + T(szPtr[2]));
			}else if (q[2] < T(szPtr[2])){
				residuals[2] = T(0.0);
			}else{
				residuals[2] = oW * (q[2] - T(szPtr[2]));
			}    

			return true;
		}
	double* observed;
};

struct AlignmentErrorFloor 
{
	AlignmentErrorFloor(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				T* residuals) const {

			T p[3];
			T q[3];   
			T r[3];

			p[0] = T(observed[2]) - camera_extrinsic[3];
			p[1] = T(observed[3]) - camera_extrinsic[4];
			p[2] = T(observed[4]) - camera_extrinsic[5];

			r[0] = - camera_extrinsic[0];
			r[1] = - camera_extrinsic[1];
			r[2] = - camera_extrinsic[2];

			ceres::AngleAxisRotatePoint(r, p, q);


			// hinge loss
			double* szPtr = object_half_size + 3 * int(observed[5]);
			T oW = T(object_weight[int(observed[5])]);

			//residuals[0] = hingeLoss<T>(q[0], T(szPtr[0]));
			//residuals[1] = hingeLoss<T>(q[1], T(szPtr[1]));
			//residuals[2] = hingeLoss<T>(q[2], T(szPtr[2])); 

			if (q[0] < T(-szPtr[0])){
				residuals[0] = oW * (q[0] + T(szPtr[0]));
			}else if (q[0] < T(szPtr[0])){
				residuals[0] = T(0.0);
			}else{
				residuals[0] = oW * (q[0] - T(szPtr[0]));
			}

			if (q[1] < T(-szPtr[1])){
				residuals[1] = oW * (q[1] + T(szPtr[1]));
			}else if (q[1] < T(szPtr[1])){
				residuals[1] = T(0.0);
			}else{
				residuals[1] = oW * (q[1] - T(szPtr[1]));
			}

			if (q[2] < T(-szPtr[2])){
				residuals[2] = oW * (q[2] + T(szPtr[2]));
			}else if (q[2] < T(szPtr[2])){
				residuals[2] = T(0.0);
			}else{
				residuals[2] = oW * (q[2] - T(szPtr[2]));
			}

			return true;
		}
	double* observed;
};

struct AlignmentErrorCeiling 
{
	AlignmentErrorCeiling(double* observed_in): observed(observed_in) {}

	template <typename T>
	bool operator()(const T* const camera_extrinsic, const T* const world2object, T* residuals) const 
	{

		T p[3];
		T q[3];   
		T r[3];

		p[0] = T(observed[2]) - camera_extrinsic[3];
		p[1] = T(observed[3]) - camera_extrinsic[4];
		p[2] = T(observed[4]) - camera_extrinsic[5];

		r[0] = - camera_extrinsic[0];
		r[1] = - camera_extrinsic[1];
		r[2] = - camera_extrinsic[2];

		ceres::AngleAxisRotatePoint(r, p, q);


		// hinge loss
		double* szPtr = object_half_size + 3 * int(observed[5]);
		T oW = T(object_weight[int(observed[5])]);

		//residuals[0] = hingeLoss<T>(q[0], T(szPtr[0]));
		//residuals[1] = hingeLoss<T>(q[1], T(szPtr[1]));
		//residuals[2] = hingeLoss<T>(q[2], T(szPtr[2])); 

		if (q[0] < T(-szPtr[0])){
			residuals[0] = oW * (q[0] + T(szPtr[0]));
		}else if (q[0] < T(szPtr[0])){
			residuals[0] = T(0.0);
		}else{
			residuals[0] = oW * (q[0] - T(szPtr[0]));
		}

		if (q[1] - world2object[0] < T(-szPtr[2])){
			residuals[1] = oW * (q[1] - world2object[0] + T(szPtr[2]));
		}else if (q[1] - world2object[0] < T(szPtr[2])){
			residuals[1] = T(0.0);
		}else{
			residuals[1] = oW * (q[1] - world2object[0] - T(szPtr[2]));
		}

		if (q[2] < T(-szPtr[1])){
			residuals[2] = oW * (q[2] + T(szPtr[1]));
		}else if (q[2] < T(szPtr[1])){
			residuals[2] = T(0.0);
		}else{
			residuals[2] = oW * (q[2] - T(szPtr[1]));
		}    

		return true;
	}
	double* observed;
};

/*
struct AlignmentErrorCameraPairs {
  AlignmentErrorCameraPairs(double* observed_in): observed(observed_in) {}

  template <typename T>
  bool operator()(const T* const cameraW2C0,const T* const cameraW2C1,T* residuals) const {
                  
    // camera_extrinsic[0,1,2] are the angle-axis rotation.
    T p[3];
    
    ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);

    
    // camera_extrinsic[3,4,5] are the translation.
    p[0] += camera_extrinsic[3];
    p[1] += camera_extrinsic[4];
    p[2] += camera_extrinsic[5];
    
    // The error is the difference between the predicted and observed position.
    residuals[2] = T(observed[5])*(p[0] - T(observed[2]))*w3Dv2D;
    residuals[3] = T(observed[5])*(p[1] - T(observed[3]))*w3Dv2D;
    residuals[4] = T(observed[5])*(p[2] - T(observed[4]))*w3Dv2D;

    // let p[2] ~= 0
    if (T(0.0)<=p[2]){
        if(p[2]<EPS){
            p[2] = EPS;
        }
    }else{
        if (p[2]>-EPS){
            p[2] = -EPS;
        }
    }
    
    // project it
    p[0] = T(fx) * p[0] / p[2] + T(px);
    p[1] = T(fy) * p[1] / p[2] + T(py);
    
    // reprojection error
    residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
    residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

    return true;
  }
  double* observed;
};

*/

struct AlignmentError2D3D 
{
	AlignmentError2D3D(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const point,
				T* residuals) const {

			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];

			ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
			/*
			   T x = camera_extrinsic[0];
			   T y = camera_extrinsic[1];
			   T z = camera_extrinsic[2];
			   T x2 = x*x;
			   T y2 = y*y;
			   T z2 = z*z;    
			   T w2 = T(1.0) - x2 - y2 - z2;
			   T w  = sqrt(w2);

			   p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
			   p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
			   p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
			   */


			// camera_extrinsic[3,4,5] are the translation.
			p[0] += camera_extrinsic[3];
			p[1] += camera_extrinsic[4];
			p[2] += camera_extrinsic[5];

			// The error is the difference between the predicted and observed position.
			residuals[2] = T(observed[5])*(p[0] - T(observed[2]))*w3Dv2D;
			residuals[3] = T(observed[5])*(p[1] - T(observed[3]))*w3Dv2D;
			residuals[4] = T(observed[5])*(p[2] - T(observed[4]))*w3Dv2D;

			//std::cout<<residuals[0]<<" "<<residuals[1]<<" "<<residuals[2]<<std::endl;
			// let p[2] ~= 0
			if (T(0.0)<=p[2]){
				if(p[2]<EPS){
					p[2] = EPS;
				}
			}else{
				if (p[2]>-EPS){
					p[2] = -EPS;
				}
			}

			// project it
			float n[4]= {0,1,0,0};
			T temp = (camera_extrinsic[3]* T(n[0]))+(camera_extrinsic[4] * T(n[1]))+(camera_extrinsic[5] * T(n[2]));

			p[0] = T(fx) * p[0] / p[2] + T(px);
			p[1] = T(fy) * p[1] / p[2] + T(py);


			//residuals[3]= T(0);
			// residuals[5] = T(observed[5])*(temp-T(n[3]))*w3Dv2D;
			//if (residuals[3] > T(0))
			//{
			//		residuals[3]=T(0);
			//}
			//std::cout<<residuals[5]<<" ";
			// reprojection error
			residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
			residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

			/*
			   if (exe_time<10){
			   exe_time ++;
			   std::cout<<"fx="<<fx<<std::endl;
			   std::cout<<"fy="<<fy<<std::endl;
			   std::cout<<"px="<<px<<std::endl;
			   std::cout<<"py="<<py<<std::endl;
			   std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
			   std::cout<<"p[0]="<<p[0]<<std::endl;
			   std::cout<<"p[1]="<<p[1]<<std::endl;
			   std::cout<<"observed[0]="<<observed[0]<<std::endl;
			   std::cout<<"observed[1]="<<observed[1]<<std::endl;
			   std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			   std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
			   std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
			   std::cout<<"residuals[3]="<<residuals[3]<<std::endl;
			   std::cout<<"residuals[4]="<<residuals[4]<<std::endl;
			   std::cout<<"--------------------------"<<std::endl;
			   }
			   */
			return true;
		}
	double* observed;
};

struct AlignmentError2D3D1 
{
	AlignmentError2D3D1(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const point,
				T* residuals) const {

			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];

			ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
			/*
			   T x = camera_extrinsic[0];
			   T y = camera_extrinsic[1];
			   T z = camera_extrinsic[2];
			   T x2 = x*x;
			   T y2 = y*y;
			   T z2 = z*z;    
			   T w2 = T(1.0) - x2 - y2 - z2;
			   T w  = sqrt(w2);

			   p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
			   p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
			   p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
			   */


			// camera_extrinsic[3,4,5] are the translation.
			p[0] += camera_extrinsic[3];
			p[1] += camera_extrinsic[4];
			p[2] += camera_extrinsic[5];

			// The error is the difference between the predicted and observed position.
			residuals[2] = T(observed[5])*(p[0] - T(observed[2]))*w3Dv2D;
			residuals[3] = T(observed[5])*(p[1] - T(observed[3]))*w3Dv2D;
			residuals[4] = T(observed[5])*(p[2] - T(observed[4]))*w3Dv2D;

			//std::cout<<residuals[0]<<" "<<residuals[1]<<" "<<residuals[2]<<std::endl;

			// let p[2] ~= 0
			if (T(0.0)<=p[2]){
				if(p[2]<EPS){
					p[2] = EPS;
				}
			}else{
				if (p[2]>-EPS){
					p[2] = -EPS;
				}
			}

			// project it
			float n[4]= {0,1,0,0};
			T temp = (camera_extrinsic[3]* T(n[0]))+(camera_extrinsic[4] * T(n[1]))+(camera_extrinsic[5] * T(n[2]));

			p[0] = T(fx) * p[0] / p[2] + T(px);
			p[1] = T(fy) * p[1] / p[2] + T(py);


			//residuals[3]= T(0);
			residuals[5] = T(observed[5])*(temp-T(n[3]))*w3Dv2D;
			//if (residuals[3] > T(0))
			//{
			//		residuals[3]=T(0);
			//}
			//std::cout<<residuals[5]<<" ";
			// reprojection error
			residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
			residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

			/*
			   if (exe_time<10){
			   exe_time ++;
			   std::cout<<"fx="<<fx<<std::endl;
			   std::cout<<"fy="<<fy<<std::endl;
			   std::cout<<"px="<<px<<std::endl;
			   std::cout<<"py="<<py<<std::endl;
			   std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
			   std::cout<<"p[0]="<<p[0]<<std::endl;
			   std::cout<<"p[1]="<<p[1]<<std::endl;
			   std::cout<<"observed[0]="<<observed[0]<<std::endl;
			   std::cout<<"observed[1]="<<observed[1]<<std::endl;
			   std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			   std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
			   std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
			   std::cout<<"residuals[3]="<<residuals[3]<<std::endl;
			   std::cout<<"residuals[4]="<<residuals[4]<<std::endl;
			   std::cout<<"--------------------------"<<std::endl;
			   }
			   */
			return true;
		}
	double* observed;
};

struct AlignmentError2D3D2 
{
	AlignmentError2D3D2(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const point,
				const T* const point1,
				T* residuals) const {

			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];

			ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
			/*
			   T x = camera_extrinsic[0];
			   T y = camera_extrinsic[1];
			   T z = camera_extrinsic[2];
			   T x2 = x*x;
			   T y2 = y*y;
			   T z2 = z*z;    
			   T w2 = T(1.0) - x2 - y2 - z2;
			   T w  = sqrt(w2);

			   p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
			   p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
			   p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
			   */


			// camera_extrinsic[3,4,5] are the translation.
			p[0] += camera_extrinsic[3];
			p[1] += camera_extrinsic[4];
			p[2] += camera_extrinsic[5];

			// The error is the difference between the predicted and observed position.
			residuals[2] = T(observed[5])*(p[0] - T(observed[2]))*w3Dv2D;
			residuals[3] = T(observed[5])*(p[1] - T(observed[3]))*w3Dv2D;
			residuals[4] = T(observed[5])*(p[2] - T(observed[4]))*w3Dv2D;

			//std::cout<<residuals[0]<<" "<<residuals[1]<<" "<<residuals[2]<<std::endl;
			// let p[2] ~= 0
			if (T(0.0)<=p[2]){
				if(p[2]<EPS){
					p[2] = EPS;
				}
			}else{
				if (p[2]>-EPS){
					p[2] = -EPS;
				}
			}

			// project it
			float n[4]= {0,1,0,0};
			T temp = (camera_extrinsic[3]* T(n[0]))+(camera_extrinsic[4] * T(n[1]))+(camera_extrinsic[5] * T(n[2]));

			p[0] = T(fx) * p[0] / p[2] + T(px);
			p[1] = T(fy) * p[1] / p[2] + T(py);


			//residuals[3]= T(0);
			// residuals[5] = T(observed[5])*(temp-T(n[3]))*w3Dv2D;
			//if (residuals[3] > T(0))
			//{
			//		residuals[3]=T(0);
			//}
			//std::cout<<residuals[5]<<" ";
			// reprojection error
			residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
			residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

			/*
			   if (exe_time<10){
			   exe_time ++;
			   std::cout<<"fx="<<fx<<std::endl;
			   std::cout<<"fy="<<fy<<std::endl;
			   std::cout<<"px="<<px<<std::endl;
			   std::cout<<"py="<<py<<std::endl;
			   std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
			   std::cout<<"p[0]="<<p[0]<<std::endl;
			   std::cout<<"p[1]="<<p[1]<<std::endl;
			   std::cout<<"observed[0]="<<observed[0]<<std::endl;
			   std::cout<<"observed[1]="<<observed[1]<<std::endl;
			   std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			   std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
			   std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
			   std::cout<<"residuals[3]="<<residuals[3]<<std::endl;
			   std::cout<<"residuals[4]="<<residuals[4]<<std::endl;
			   std::cout<<"--------------------------"<<std::endl;
			   }
			   */
			return true;
		}
	double* observed;
};


struct AlignmentErrortran 
{
	AlignmentErrortran(double* observed_in): observed(observed_in) {}

	template <typename T>
		bool operator()(const T* const camera_extrinsic,
				const T* const camera_extrinsic1,
				const T* const camera_extrinsic2,
				T* residuals) const {

			// camera_extrinsic[0,1,2] are the angle-axis rotation.
			T p[3];
			//  int camera_extrinsic[3]={0,0,0};
			//  ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
			/*
			   T x = camera_extrinsic[0];
			   T y = camera_extrinsic[1];
			   T z = camera_extrinsic[2];
			   T x2 = x*x;
			   T y2 = y*y;
			   T z2 = z*z;    
			   T w2 = T(1.0) - x2 - y2 - z2;
			   T w  = sqrt(w2);

			   p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
			   p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
			   p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
			   */




			// camera_extrinsic[3,4,5] are the translation.
			//  p[0] += camera_extrinsic[3];
			// p[1] += camera_extrinsic[4];
			//p[2] += camera_extrinsic[5];

			// The error is the difference between the predicted and observed position.
			// residuals[2] = T(observed[5])*(p[0] - T(observed[2]))*w3Dv2D;
			//residuals[3] = T(observed[5])*(p[1] - T(observed[3]))*w3Dv2D;
			// residuals[4] = T(observed[5])*(p[2] - T(observed[4]))*w3Dv2D;

			// let p[2] ~= 0
			/*  if (T(0.0)<=p[2]){
				if(p[2]<EPS){
				p[2] = EPS;
				}
				}else{
				if (p[2]>-EPS){
				p[2] = -EPS;
				}
				}

			// project it
			p[0] = T(fx) * p[0] / p[2] + T(px);
			p[1] = T(fy) * p[1] / p[2] + T(py);*/

			// reprojection error

			T temp[3],temp1[3];

			temp[0] = -(camera_extrinsic[5]*camera_extrinsic2[4]) + (camera_extrinsic[4]*camera_extrinsic2[5]);
			temp[1] = (camera_extrinsic[5]*camera_extrinsic2[3]) - (camera_extrinsic[3]*camera_extrinsic2[5]);
			temp[2] = -(camera_extrinsic[4]*camera_extrinsic2[3])+(camera_extrinsic[3]*camera_extrinsic2[4]);

			temp1[0] = -(camera_extrinsic1[5]*camera_extrinsic2[4]) + (camera_extrinsic1[4]*camera_extrinsic2[5]);
			temp1[1] = (camera_extrinsic1[5]*camera_extrinsic2[3]) - (camera_extrinsic1[3]*camera_extrinsic2[5]);
			temp1[2] = -(camera_extrinsic1[4]*camera_extrinsic2[3])+(camera_extrinsic1[3]*camera_extrinsic2[4]);

			residuals[0] = T(observed[5])*(temp[0] - temp1[0]);
			residuals[1] = T(observed[5])*(temp[1] - temp1[1]);
			residuals[2] = T(observed[5])*(temp[2] - temp1[2]);
			//  std::cout<<residuals[0]<<" "<<residuals[1]<<" "<<residuals[2]<<std::endl;

			float n[4]= {0,1,0,1};
			// residuals[0] = T(observed[5])*(p[0]*n[0]+p[1]*n[1]+p[2]*n[2]-n[3]);
			// residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

			/*
			   if (exe_time<10){
			   exe_time ++;
			   std::cout<<"fx="<<fx<<std::endl;
			   std::cout<<"fy="<<fy<<std::endl;
			   std::cout<<"px="<<px<<std::endl;
			   std::cout<<"py="<<py<<std::endl;
			   std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
			   std::cout<<"p[0]="<<p[0]<<std::endl;
			   std::cout<<"p[1]="<<p[1]<<std::endl;
			   std::cout<<"observed[0]="<<observed[0]<<std::endl;
			   std::cout<<"observed[1]="<<observed[1]<<std::endl;
			   std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			   std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
			   std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
			   std::cout<<"residuals[3]="<<residuals[3]<<std::endl;
			   std::cout<<"residuals[4]="<<residuals[4]<<std::endl;
			   std::cout<<"--------------------------"<<std::endl;
			   }
			   */
			return true;
		}
	double* observed;
};


struct AlignmentErrorbox_new 
{
	AlignmentErrorbox_new(double* observed_in): observed(observed_in) {}


	template <typename T>
	bool operator()(const T* const c, 
					const T* const camera_extrinsic, 
					const T* const point, 
					const T* const point2, 
						  T* residuals) const 
	{
		/*             
		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T p[3];
		T p1[3];    


		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
		ceres::AngleAxisRotatePoint(camera_extrinsic, point2, p1);


		//T x = camera_extrinsic[0];
		//T y = camera_extrinsic[1];
		//T z = camera_extrinsic[2];
		//T x2 = x*x;
		//T y2 = y*y;
		//T z2 = z*z;    
		//T w2 = T(1.0) - x2 - y2 - z2;
		//T w  = sqrt(w2);

		//p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
		//p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
		//p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);



		// camera_extrinsic[3,4,5] are the translation.
		p[0] += camera_extrinsic[3];
		p[1] += camera_extrinsic[4];
		p[2] += camera_extrinsic[5];

		// camera_extrinsic[3,4,5] are the translation.
		p1[0] += camera_extrinsic[3];
		p1[1] += camera_extrinsic[4];
		p1[2] += camera_extrinsic[5];

		// The error is the difference between the predicted and observed position.
		residuals[0] = p[0] - p1[0]-c[0];
		residuals[1] = p[1] - p1[1]-c[1];
		residuals[2] = p[2] - p1[2]-c[2];

		//std::cout<<residuals[0]<<" "<<residuals[1]<<" "<<residuals[2]<<std::endl;

		// let p[2] ~= 0
		if (T(0.0)<=p[2]){
			if(p[2]<EPS){
				p[2] = EPS;
			}
		}else{
			if (p[2]>-EPS){
				p[2] = -EPS;
			}
		}
		*/


			//if (exe_time<10){
			//exe_time ++;
			//std::cout<<"fx="<<fx<<std::endl;
			//std::cout<<"fy="<<fy<<std::endl;
			//std::cout<<"px="<<px<<std::endl;
			//std::cout<<"py="<<py<<std::endl;
			//std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
			//std::cout<<"p[0]="<<p[0]<<std::endl;
			//std::cout<<"p[1]="<<p[1]<<std::endl;
			//std::cout<<"observed[0]="<<observed[0]<<std::endl;
			//std::cout<<"observed[1]="<<observed[1]<<std::endl;
			//std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
			//std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
			//std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
			//std::cout<<"residuals[3]="<<residuals[3]<<std::endl;
			//std::cout<<"--------------------------"<<std::endl;
			//}

			return true;
	}
	double* observed;
};


struct AlignmentErrorbox_new1 
{
	AlignmentErrorbox_new1(double* observed_in): observed(observed_in) {}
	template <typename T>

	bool operator()(const T* const c,
					const T* const point,
					const T* const point2,
					const T* const camera_extrinsic, 
						  T* residuals) const 
	{

		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T p[3];
		T p1[3];    

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
		ceres::AngleAxisRotatePoint(camera_extrinsic, point2, p1);

		//T x = camera_extrinsic[0];
		//T y = camera_extrinsic[1];
		//T z = camera_extrinsic[2];
		//T x2 = x*x;
		//T y2 = y*y;
		//T z2 = z*z;    
		//T w2 = T(1.0) - x2 - y2 - z2;
		//T w  = sqrt(w2);

		//p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
		//p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
		//p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);

		// camera_extrinsic[3,4,5] are the translation.
		p[0] += camera_extrinsic[3];
		p[1] += camera_extrinsic[4];
		p[2] += camera_extrinsic[5];

		// camera_extrinsic[3,4,5] are the translation.
		p1[0] += camera_extrinsic[3];
		p1[1] += camera_extrinsic[4];
		p1[2] += camera_extrinsic[5];
		//std::cout<<p[0]<<" "<<p1[0]<<std::endl;
		//// The error is the difference between the predicted and observed position.

		// residuals[0] = (p[1] - p1[1] - c[0]) ; //temp[0] - temp1[0]);
		// residuals[0] =  (sqrt(pow(p[0] - p1[0],2)+pow(p[1] - p1[1],2)+pow(p[2] - p1[2],2))-c[0]);
		// residuals[0] =  (sqrt(pow(p[0] - p1[0] - c[0],2)+pow(p[1] - p1[1] - c[1],2)+pow(p[2] - p1[2] - c[2],2)));
		residuals[0] = (p[0]-p1[0]-c[0]);
		residuals[1] = (p[1]-p1[1]-c[1]);
		residuals[2] = (p[2]-p1[2]-c[2]);
		// std:
		// residuals[1] = (p[1] - p1[1] - c[1]);//(p[0] - p1[0] + c);
		// residuals[2] = (p[2] - p1[2] - c[2]); //(p[0] - p1[0] + c);

		// std::cout<<"Outputting c[0]"<<c[0]<<" "<<std::endl;

		//// let p[2] ~= 0
		//if (T(0.0)<=p[2]){
		//if(p[2]<EPS){
		//p[2] = EPS;
		//}
		//}else{
		//if (p[2]>-EPS){
		//p[2] = -EPS;
		//}
		//}

		////if (exe_time<10){
		//exe_time ++;
		//std::cout<<"fx="<<fx<<std::endl;
		//std::cout<<"fy="<<fy<<std::endl;
		//std::cout<<"px="<<px<<std::endl;
		//std::cout<<"py="<<py<<std::endl;
		//std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
		//std::cout<<"p[0]="<<p[0]<<std::endl;
		//std::cout<<"p[1]="<<p[1]<<std::endl;
		//std::cout<<"p1[0]="<<p1[0]<<std::endl;
		//std::cout<<"p1[1]="<<p1[1]<<std::endl;
		//std::cout<<"observed[0]="<<observed[0]<<std::endl;
		//std::cout<<"observed[1]="<<observed[1]<<std::endl;
		//std::cout<<"residuals[0][0]="<<residuals[0]<<std::endl;
		//std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
		//std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
		//std::cout<<"--------------------------"<<std::endl;
		//}

		return true;
	}
	double* observed;

};
 

void ceres_fill_camera_parameters( double *camera_matrix, double *camera_parameter, int n_cameras );

void ceres_fill_object_parameters( double *object_matrix, double *object_parameter, unsigned int *object_type, int n_objects );

void ceres_add_triangulation_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_alignment_error_2d_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_alignment_error_3d_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_alignment_error_2d3d_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_translation_error_function( double *obs_ptr, double *camera_ptr, double *cam_ptr_one, double *cam_ptr_two, 
										   ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_alignment_traj_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame );

void ceres_add_alignment_traj_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
											  double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame );

void ceres_add_alignment_traj_normal_box_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_cloud,
														 double *cam_ptr_one, double *cam_ptr_two, unsigned int *point_observed_index, 
														 ceres::LossFunction *loss_function, ceres::Problem &problem, 
														 unsigned int *random_array, double *box_constraints_var, int n_points, int n_camera, 
														 int n_rand_pairs, int id_obs, int not_first_frame );

#endif /* ifndef CERES_FUNCTIONS_H */


