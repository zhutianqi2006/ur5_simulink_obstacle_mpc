/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState ee_px;
    DifferentialState ee_py;
    DifferentialState ee_pz;
    DifferentialState quat_ex;
    DifferentialState quat_ey;
    DifferentialState quat_ez;
    DifferentialState p1_px;
    DifferentialState p1_py;
    DifferentialState p1_pz;
    DifferentialState p2_px;
    DifferentialState p2_py;
    DifferentialState p2_pz;
    DifferentialState p3_px;
    DifferentialState p3_py;
    DifferentialState p3_pz;
    DifferentialState p4_px;
    DifferentialState p4_py;
    DifferentialState p4_pz;
    DifferentialState p5_px;
    DifferentialState p5_py;
    DifferentialState p5_pz;
    DifferentialState p6_px;
    DifferentialState p6_py;
    DifferentialState p6_pz;
    DifferentialState obj1_px;
    DifferentialState obj1_py;
    DifferentialState obj1_pz;
    DifferentialState obj1_vx;
    DifferentialState obj1_vy;
    DifferentialState obj1_vz;
    Control joint1_v;
    Control joint2_v;
    Control joint3_v;
    Control joint4_v;
    Control joint5_v;
    Control joint6_v;
    OnlineData jp1_11; 
    OnlineData jp1_12; 
    OnlineData jp1_21; 
    OnlineData jp1_22; 
    OnlineData jp1_31; 
    OnlineData jp1_32; 
    OnlineData jp2_11; 
    OnlineData jp2_12; 
    OnlineData jp2_21; 
    OnlineData jp2_22; 
    OnlineData jp2_31; 
    OnlineData jp2_32; 
    OnlineData jp3_11; 
    OnlineData jp3_12; 
    OnlineData jp3_21; 
    OnlineData jp3_22; 
    OnlineData jp3_31; 
    OnlineData jp3_32; 
    OnlineData jp4_11; 
    OnlineData jp4_12; 
    OnlineData jp4_13; 
    OnlineData jp4_21; 
    OnlineData jp4_22; 
    OnlineData jp4_23; 
    OnlineData jp4_31; 
    OnlineData jp4_32; 
    OnlineData jp4_33; 
    OnlineData jp5_11; 
    OnlineData jp5_12; 
    OnlineData jp5_13; 
    OnlineData jp5_21; 
    OnlineData jp5_22; 
    OnlineData jp5_23; 
    OnlineData jp5_31; 
    OnlineData jp5_32; 
    OnlineData jp5_33; 
    OnlineData jp6_11; 
    OnlineData jp6_12; 
    OnlineData jp6_13; 
    OnlineData jp6_14; 
    OnlineData jp6_15; 
    OnlineData jp6_16; 
    OnlineData jp6_21; 
    OnlineData jp6_22; 
    OnlineData jp6_23; 
    OnlineData jp6_24; 
    OnlineData jp6_25; 
    OnlineData jp6_26; 
    OnlineData jp6_31; 
    OnlineData jp6_32; 
    OnlineData jp6_33; 
    OnlineData jp6_34; 
    OnlineData jp6_35; 
    OnlineData jp6_36; 
    OnlineData jee_11; 
    OnlineData jee_12; 
    OnlineData jee_13; 
    OnlineData jee_14; 
    OnlineData jee_15; 
    OnlineData jee_16; 
    OnlineData jee_21; 
    OnlineData jee_22; 
    OnlineData jee_23; 
    OnlineData jee_24; 
    OnlineData jee_25; 
    OnlineData jee_26; 
    OnlineData jee_31; 
    OnlineData jee_32; 
    OnlineData jee_33; 
    OnlineData jee_34; 
    OnlineData jee_35; 
    OnlineData jee_36; 
    OnlineData jee_41; 
    OnlineData jee_42; 
    OnlineData jee_43; 
    OnlineData jee_44; 
    OnlineData jee_45; 
    OnlineData jee_46; 
    OnlineData jee_51; 
    OnlineData jee_52; 
    OnlineData jee_53; 
    OnlineData jee_54; 
    OnlineData jee_55; 
    OnlineData jee_56; 
    OnlineData jee_61; 
    OnlineData jee_62; 
    OnlineData jee_63; 
    OnlineData jee_64; 
    OnlineData jee_65; 
    OnlineData jee_66; 
    DVector acadodata_v1(3);
    acadodata_v1(0) = 0;
    acadodata_v1(1) = 4.000000E-02;
    acadodata_v1(2) = 2.000000E-01;
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << ee_px;
    acadodata_f2 << ee_py;
    acadodata_f2 << ee_pz;
    acadodata_f2 << quat_ex;
    acadodata_f2 << quat_ey;
    acadodata_f2 << quat_ez;
    acadodata_f2 << p1_px;
    acadodata_f2 << p1_py;
    acadodata_f2 << p1_pz;
    acadodata_f2 << p2_px;
    acadodata_f2 << p2_py;
    acadodata_f2 << p2_pz;
    acadodata_f2 << p3_px;
    acadodata_f2 << p3_py;
    acadodata_f2 << p3_pz;
    acadodata_f2 << p4_px;
    acadodata_f2 << p4_py;
    acadodata_f2 << p4_pz;
    acadodata_f2 << p5_px;
    acadodata_f2 << p5_py;
    acadodata_f2 << p5_pz;
    acadodata_f2 << p6_px;
    acadodata_f2 << p6_py;
    acadodata_f2 << p6_pz;
    acadodata_f2 << obj1_px;
    acadodata_f2 << obj1_py;
    acadodata_f2 << obj1_pz;
    acadodata_f2 << obj1_vx;
    acadodata_f2 << obj1_vy;
    acadodata_f2 << obj1_vz;
    acadodata_f2 << joint1_v;
    acadodata_f2 << joint2_v;
    acadodata_f2 << joint3_v;
    acadodata_f2 << joint4_v;
    acadodata_f2 << joint5_v;
    acadodata_f2 << joint6_v;
    Function acadodata_f3;
    acadodata_f3 << ee_px;
    acadodata_f3 << ee_py;
    acadodata_f3 << ee_pz;
    acadodata_f3 << quat_ex;
    acadodata_f3 << quat_ey;
    acadodata_f3 << quat_ez;
    acadodata_f3 << p1_px;
    acadodata_f3 << p1_py;
    acadodata_f3 << p1_pz;
    acadodata_f3 << p2_px;
    acadodata_f3 << p2_py;
    acadodata_f3 << p2_pz;
    acadodata_f3 << p3_px;
    acadodata_f3 << p3_py;
    acadodata_f3 << p3_pz;
    acadodata_f3 << p4_px;
    acadodata_f3 << p4_py;
    acadodata_f3 << p4_pz;
    acadodata_f3 << p5_px;
    acadodata_f3 << p5_py;
    acadodata_f3 << p5_pz;
    acadodata_f3 << p6_px;
    acadodata_f3 << p6_py;
    acadodata_f3 << p6_pz;
    acadodata_f3 << obj1_px;
    acadodata_f3 << obj1_py;
    acadodata_f3 << obj1_pz;
    acadodata_f3 << obj1_vx;
    acadodata_f3 << obj1_vy;
    acadodata_f3 << obj1_vz;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(ee_px) == (jee_11*joint1_v+jee_12*joint2_v+jee_13*joint3_v+jee_14*joint4_v+jee_15*joint5_v+jee_16*joint6_v);
    acadodata_f1 << dot(ee_py) == (jee_21*joint1_v+jee_22*joint2_v+jee_23*joint3_v+jee_24*joint4_v+jee_25*joint5_v+jee_26*joint6_v);
    acadodata_f1 << dot(ee_pz) == (jee_31*joint1_v+jee_32*joint2_v+jee_33*joint3_v+jee_34*joint4_v+jee_35*joint5_v+jee_36*joint6_v);
    acadodata_f1 << dot(quat_ex) == (jee_41*joint1_v+jee_42*joint2_v+jee_43*joint3_v+jee_44*joint4_v+jee_45*joint5_v+jee_46*joint6_v);
    acadodata_f1 << dot(quat_ey) == (jee_51*joint1_v+jee_52*joint2_v+jee_53*joint3_v+jee_54*joint4_v+jee_55*joint5_v+jee_56*joint6_v);
    acadodata_f1 << dot(quat_ez) == (jee_61*joint1_v+jee_62*joint2_v+jee_63*joint3_v+jee_64*joint4_v+jee_65*joint5_v+jee_66*joint6_v);
    acadodata_f1 << dot(p1_px) == (joint1_v*jp1_11+joint2_v*jp1_12);
    acadodata_f1 << dot(p1_py) == (joint1_v*jp1_21+joint2_v*jp1_22);
    acadodata_f1 << dot(p1_pz) == (joint1_v*jp1_31+joint2_v*jp1_32);
    acadodata_f1 << dot(p2_px) == (joint1_v*jp2_11+joint2_v*jp2_12);
    acadodata_f1 << dot(p2_py) == (joint1_v*jp2_21+joint2_v*jp2_22);
    acadodata_f1 << dot(p2_pz) == (joint1_v*jp2_31+joint2_v*jp2_32);
    acadodata_f1 << dot(p3_px) == (joint1_v*jp3_11+joint2_v*jp3_12);
    acadodata_f1 << dot(p3_py) == (joint1_v*jp3_21+joint2_v*jp3_22);
    acadodata_f1 << dot(p3_pz) == (joint1_v*jp3_31+joint2_v*jp3_32);
    acadodata_f1 << dot(p4_px) == (joint1_v*jp4_11+joint2_v*jp4_12+joint3_v*jp4_13);
    acadodata_f1 << dot(p4_py) == (joint1_v*jp4_21+joint2_v*jp4_22+joint3_v*jp4_23);
    acadodata_f1 << dot(p4_pz) == (joint1_v*jp4_31+joint2_v*jp4_32+joint3_v*jp4_33);
    acadodata_f1 << dot(p5_px) == (joint1_v*jp5_11+joint2_v*jp5_12+joint3_v*jp5_13);
    acadodata_f1 << dot(p5_py) == (joint1_v*jp5_21+joint2_v*jp5_22+joint3_v*jp5_23);
    acadodata_f1 << dot(p5_pz) == (joint1_v*jp5_31+joint2_v*jp5_32+joint3_v*jp5_33);
    acadodata_f1 << dot(p6_px) == (joint1_v*jp6_11+joint2_v*jp6_12+joint3_v*jp6_13+joint4_v*jp6_14+joint5_v*jp6_15+joint6_v*jp6_16);
    acadodata_f1 << dot(p6_py) == (joint1_v*jp6_21+joint2_v*jp6_22+joint3_v*jp6_23+joint4_v*jp6_24+joint5_v*jp6_25+joint6_v*jp6_26);
    acadodata_f1 << dot(p6_pz) == (joint1_v*jp6_31+joint2_v*jp6_32+joint3_v*jp6_33+joint4_v*jp6_34+joint5_v*jp6_35+joint6_v*jp6_36);
    acadodata_f1 << dot(obj1_px) == obj1_vx;
    acadodata_f1 << dot(obj1_py) == obj1_vy;
    acadodata_f1 << dot(obj1_pz) == obj1_vz;
    acadodata_f1 << dot(obj1_vx) == 0.00000000000000000000e+00;
    acadodata_f1 << dot(obj1_vy) == 0.00000000000000000000e+00;
    acadodata_f1 << dot(obj1_vz) == 0.00000000000000000000e+00;

    Grid grid_acadodata_v1(acadodata_v1);
    OCP ocp1(grid_acadodata_v1);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= joint1_v <= 5.99999999999999977796e-01);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= joint2_v <= 5.99999999999999977796e-01);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= joint3_v <= 5.99999999999999977796e-01);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= joint4_v <= 5.99999999999999977796e-01);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= joint5_v <= 5.99999999999999977796e-01);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= joint6_v <= 5.99999999999999977796e-01);
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((-obj1_px+p1_px),2.00000000000000000000e+00)+pow((-obj1_py+p1_py),2.00000000000000000000e+00)+pow((-obj1_pz+p1_pz),2.00000000000000000000e+00))));
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((-obj1_px+p2_px),2.00000000000000000000e+00)+pow((-obj1_py+p2_py),2.00000000000000000000e+00)+pow((-obj1_pz+p2_pz),2.00000000000000000000e+00))));
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((-obj1_px+p3_px),2.00000000000000000000e+00)+pow((-obj1_py+p3_py),2.00000000000000000000e+00)+pow((-obj1_pz+p3_pz),2.00000000000000000000e+00))));
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((-obj1_px+p4_px),2.00000000000000000000e+00)+pow((-obj1_py+p4_py),2.00000000000000000000e+00)+pow((-obj1_pz+p4_pz),2.00000000000000000000e+00))));
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((-obj1_px+p5_px),2.00000000000000000000e+00)+pow((-obj1_py+p5_py),2.00000000000000000000e+00)+pow((-obj1_pz+p5_pz),2.00000000000000000000e+00))));
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((-obj1_px+p6_px),2.00000000000000000000e+00)+pow((-obj1_py+p6_py),2.00000000000000000000e+00)+pow((-obj1_pz+p6_pz),2.00000000000000000000e+00))));
    ocp1.subjectTo(2.50000000000000000000e-01 <= sqrt((pow((ee_px-obj1_px),2.00000000000000000000e+00)+pow((ee_py-obj1_py),2.00000000000000000000e+00)+pow((ee_pz-obj1_pz),2.00000000000000000000e+00))));
    ocp1.setModel( acadodata_f1 );


    ocp1.setNU( 6 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 90 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 1 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

