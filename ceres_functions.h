/* ceres_functions.h
 *
 * Copyright (c)2015 Visesh Chari <visesh [at] research.iiit.net>
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
#include "vector_utils.h"

using namespace std;
#define EPS T(0.00001)

extern double fx, fy, px, py, w3Dv2D;
extern double *object_half_size, *object_weight;
extern double N[4];

template <class T>
void set_residual_hinge_loss( T q, T sz_ptr, T obj_wt, T &residual )
{
    if( q < -sz_ptr )
        residual = obj_wt * ( q + sz_ptr );
    else if( q < sz_ptr )
        residual = T( 0.0 );
    else
        residual = obj_wt * ( q - sz_ptr );
    return ;
}

template <class T>
void set_projection_precision( T *p )
{
    if( p[2] >= T(0.0) ){ if( p[2] < EPS ) p[2] = EPS; }
    else if( p[2] > -EPS ) p[2] = -EPS;
}

template <class T>
void set_camera_extrinsics( T *p, const T *camera_extrinsic, bool pcorrect = true )
{
    vector_plusequal( p, camera_extrinsic );

    if( pcorrect ) set_projection_precision( p );

    return;
}

template <class T>
void project_point( T *p, T fx, T fy, T px, T py )
{
    p[0] = fx * p[0] / p[2] + px;
    p[1] = fy * p[1] / p[2] + py;
}
 
struct AlignmentErrorTriangulate 
{
	AlignmentErrorTriangulate(double* observed_in, double* camera_extrinsic_in): observed(observed_in), camera_extrinsic(camera_extrinsic_in) {}

	template <typename T>
	bool operator()(const T* const point, T* residuals) const 
	{
		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T *p = new T[3]();

		ceres::AngleAxisRotatePoint((T*)(camera_extrinsic), point, p);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, (T *)( camera_extrinsic ) + 3 );

		// project it
        project_point( p, (T) fx, (T) fy, (T) px, (T) py );

		// reprojection error
        vector_subtract( residuals, p, (T *) observed, 2 );

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
		T *p = new T[3]();

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3 );

		// project it
        project_point( p, (T) fx, (T) fy, (T) px, (T) py );

		// reprojection error
        vector_scalar_mult_subtract( residuals, p, observed, (T) observed[5], 2 );

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
		T *p = new T[3]();

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3, false );

		// The error is the difference between the predicted and observed position.
        vector_scalar_mult_subtract( residuals, p, observed + 2, (T) observed[5] );

		return true;
	}
	double* observed;
};

struct AlignmentErrorBox 
{
	AlignmentErrorBox(double* observed_in): observed(observed_in) {}

	template <typename T>
	bool operator()(const T* const camera_extrinsic,
			const T* const world2object,
			T* residuals) const {

		T *p = new T[3]();
		T *q = new T[3]();  
		T *o = new T[3]();  
		T *r = new T[3]();

        // vector_set_equal( p, vector_subtract( (T *) observed + 2, camera_extrinsic + 3 ) );
        vector_subtract( p, (T *) observed + 2, camera_extrinsic + 3 );

        vector_minusequal( r, camera_extrinsic );
        /*
		r[0] = -camera_extrinsic[0];
		r[1] = -camera_extrinsic[1];
		r[2] = -camera_extrinsic[2];
        */

		ceres::AngleAxisRotatePoint(r, p, o);

		ceres::AngleAxisRotatePoint(world2object, o, q);

        vector_plusequal( q, world2object + 3 );

		// hinge loss
		double* szPtr = object_half_size + 3 * int(observed[5]);
		T oW = T(object_weight[int(observed[5])]);

        set_residual_hinge_loss( q[0], T( szPtr[0] ), oW, residuals[0] );
        set_residual_hinge_loss( q[1], T( szPtr[1] ), oW, residuals[1] );
        set_residual_hinge_loss( q[2], T( szPtr[2] ), oW, residuals[2] );

		return true;
	}
	double* observed;
};

struct AlignmentErrorAxisManhattanBox 
{
	AlignmentErrorAxisManhattanBox(double* observed_in): observed(observed_in) {}
	template <typename T>
	bool operator()(const T* const camera_extrinsic,const T* const world2object,T* residuals) const {
		T *p = new T[3]();
		T *q = new T[3]();  
		T *o = new T[3]();  
		T *r = new T[3]();

        vector_subtract( p, (T *) observed + 2, camera_extrinsic + 3 );
        vector_minusequal( r, camera_extrinsic );

		ceres::AngleAxisRotatePoint(r, p, o);

        vector_add( q, o, world2object );

		double* szPtr = object_half_size + 3 * int(observed[5]);
		T oW = T(object_weight[int(observed[5])]);
        set_residual_hinge_loss( q[0], T( szPtr[0] ), oW, residuals[0] );
        set_residual_hinge_loss( q[1], T( szPtr[1] ), oW, residuals[1] );
        set_residual_hinge_loss( q[2], T( szPtr[2] ), oW, residuals[2] );

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

		T *p = new T[3]();
		T *q = new T[3]();  
		T *o = new T[3]();  
		T *r = new T[3]();

        vector_subtract( p, (T *) observed + 2, camera_extrinsic + 3 );
        vector_minusequal( r, camera_extrinsic );

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
        set_residual_hinge_loss( q[0], T( szPtr[0] ), oW, residuals[0] );
        set_residual_hinge_loss( q[1], T( szPtr[1] ), oW, residuals[1] );
        set_residual_hinge_loss( q[2], T( szPtr[2] ), oW, residuals[2] );

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

		T *p = new T[3]();
		T *q = new T[3]();   
		T *r = new T[3]();

        vector_subtract( p, (T *) observed + 2, camera_extrinsic + 3 );
        vector_minusequal( r, camera_extrinsic );

		ceres::AngleAxisRotatePoint(r, p, q);

		// hinge loss
		double* szPtr = object_half_size + 3 * int(observed[5]);
		T oW = T(object_weight[int(observed[5])]);
        set_residual_hinge_loss( q[0], T( szPtr[0] ), oW, residuals[0] );
        set_residual_hinge_loss( q[1], T( szPtr[1] ), oW, residuals[1] );
        set_residual_hinge_loss( q[2], T( szPtr[2] ), oW, residuals[2] );

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

		T *p = new T[3]();
		T q[3];   
		T r[3];

        vector_subtract( p, (T *) observed + 2, camera_extrinsic + 3 );
        vector_minusequal( r, camera_extrinsic );

		ceres::AngleAxisRotatePoint(r, p, q);

		// hinge loss
		double* szPtr = object_half_size + 3 * int(observed[5]);
		T oW = T(object_weight[int(observed[5])]);

        set_residual_hinge_loss( q[0], T( szPtr[0] ), oW, residuals[0] );

		if (q[1] - world2object[0] < T(-szPtr[2]))
			residuals[1] = oW * (q[1] - world2object[0] + T(szPtr[2]));
		else if (q[1] - world2object[0] < T(szPtr[2]))
			residuals[1] = T(0.0);
		else
			residuals[1] = oW * (q[1] - world2object[0] - T(szPtr[2]));
		

        set_residual_hinge_loss( q[2], T( szPtr[1] ), oW, residuals[2] );

		return true;
	}
	double* observed;
};


struct AlignmentError2D3D 
{
	AlignmentError2D3D(double* observed_in): observed(observed_in) {}

	template <typename T>
	bool operator()(const T* const camera_extrinsic,
			const T* const point,
			T* residuals) const 
    {

		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T *p = new T[3]();

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3, false );

		// The error is the difference between the predicted and observed position.
        // vector_scalar_mult_subtract( residuals + 2, p, observed + 2, ( (T) observed[5] ) * w3Dv2D );
        vector_scalar_mult_subtract( residuals + 2, p, observed + 2, (T) observed[5] * w3Dv2D );

        set_projection_precision( p );

		// project it
        project_point( p, (T) fx, (T) fy, (T) px, (T) py );

		// reprojection error
        vector_scalar_mult_subtract( residuals, p, observed, (T) observed[5], 2 );
		// residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
		// residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

		return true;
	}
	double* observed;
};

struct AlignmentError2D3D1 
{
	AlignmentError2D3D1(double* observed_in): observed(observed_in) {}

	template <typename T>
	bool operator()(const T* const camera_extrinsic,
			const T* const point,/*const T* const normal,*/
			T* residuals) const {

		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T *p = new T[3]();

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3, false ) ;

		// The error is the difference between the predicted and observed position.
        vector_scalar_mult_subtract( residuals + 2, p, observed + 2, (T) observed[5] * w3Dv2D );

		// project it
	//	float n[4]= {0,1,0,0};
		T temp = (camera_extrinsic[3]* T(N[0]))+(camera_extrinsic[4] * T(N[1]))+(camera_extrinsic[5] * T(N[2]));

        project_point( p, (T) fx, (T) fy, (T) px, (T) py );

		//residuals[3]= T(0);
		residuals[5] = T(observed[5])*(temp-T(N[3]))*w3Dv2D;

		// reprojection error
		residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
		residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

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
		T *p = new T[3]();

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3 );

		// The error is the difference between the predicted and observed position.
        vector_scalar_mult_subtract( residuals + 2, p, observed + 2, (T) observed[5] * w3Dv2D );

        set_projection_precision( p );

		// project it
        project_point( p, (T) fx, (T) fy, (T) px, (T) py );

		residuals[0] = T(observed[5])*(p[0] - T(observed[0]));
		residuals[1] = T(observed[5])*(p[1] - T(observed[1]));     

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
		T *p = new T[3]();

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

		return true;
	}
	double* observed;
};

struct AlignmentErrortran1 
{
	AlignmentErrortran1(double* observed_in): observed(observed_in) {}

	template <typename T>
	bool operator()(const T* const camera_extrinsic,
			const T* const camera_extrinsic1,
			const T* const camera_extrinsic2,
			T* residuals) const {

		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T *p = new T[3]();

		// reprojection error
		T temp[3],temp1[3];
/*
		temp[0] = -(camera_extrinsic[5]*camera_extrinsic2[4]) + (camera_extrinsic[4]*camera_extrinsic2[5]);
		temp[1] = (camera_extrinsic[5]*camera_extrinsic2[3]) - (camera_extrinsic[3]*camera_extrinsic2[5]);
		temp[2] = -(camera_extrinsic[4]*camera_extrinsic2[3])+(camera_extrinsic[3]*camera_extrinsic2[4]);

		temp1[0] = -(camera_extrinsic1[5]*camera_extrinsic2[4]) + (camera_extrinsic1[4]*camera_extrinsic2[5]);
		temp1[1] = (camera_extrinsic1[5]*camera_extrinsic2[3]) - (camera_extrinsic1[3]*camera_extrinsic2[5]);
		temp1[2] = -(camera_extrinsic1[4]*camera_extrinsic2[3])+(camera_extrinsic1[3]*camera_extrinsic2[4]);*/

		residuals[0] = T(observed[5])*(camera_extrinsic[3] - camera_extrinsic1[3]);
		residuals[1] = T(observed[5])*(camera_extrinsic[4] - camera_extrinsic1[4]);
		residuals[2] = T(observed[5])*(camera_extrinsic[5] - camera_extrinsic1[5]);

		return true;
	}
	double* observed;
};



struct AlignmentErrorbox_new 
{
	AlignmentErrorbox_new(double* observed_in): observed(observed_in) {}
	template <typename T>

	bool operator()(const T* const c,
					const T* const point,
					const T* const point2,
					const T* const camera_extrinsic, 
						  T* residuals) const 
	{

		// camera_extrinsic[0,1,2] are the angle-axis rotation.
		T *p = new T[3]();
		T *p1 = new T[3]();    

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
		ceres::AngleAxisRotatePoint(camera_extrinsic, point2, p1);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3, false );
        set_camera_extrinsics( p1, camera_extrinsic + 3, false );

        vector_subtract( residuals, p, p1 );
        vector_minusequal_scalar( residuals, *c );

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
		T *p = new T[3]();
		T *p1 = new T[3]();    

		ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
		ceres::AngleAxisRotatePoint(camera_extrinsic, point2, p1);

		// camera_extrinsic[3,4,5] are the translation.
        set_camera_extrinsics( p, camera_extrinsic + 3, false );
        set_camera_extrinsics( p1, camera_extrinsic + 3, false );

        vector_subtract( residuals, p, p1 );
        vector_minusequal( residuals, c );

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

void ceres_add_translation1_error_function( double *obs_ptr, double *camera_ptr, double *cam_ptr_one, double *cam_ptr_two, 
										   ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_alignment_traj1_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame/*, double n[]*/ );

void ceres_add_alignment_traj1_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
											  double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame );


void ceres_add_alignment_traj_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame/*, double n[]*/ );

void ceres_add_alignment_traj_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
											  double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame );

unsigned int ceres_add_alignment_traj_normal_box_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_cloud,
														 double *cam_ptr_one, double *cam_ptr_two, unsigned int *point_observed_index, 
														 ceres::LossFunction *loss_function, ceres::Problem &problem, 
														 unsigned int *random_array, double *box_constraints_var, int n_points, int n_camera, 
														 int n_rand_pairs, int id_obs, int mode, int not_first_frame );

void ceres_add_box_constraints_scalar( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_box_constraints_vector( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_box_constraints_partial( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem );

void ceres_add_box_constraints_all( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem );

void generate_random_number_array( unsigned int *random_array, int n_rand_points, int max_pts = 100 );
#endif /* ifndef CERES_FUNCTIONS_H */
