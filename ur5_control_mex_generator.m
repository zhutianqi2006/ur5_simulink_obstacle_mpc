clc;
clear all;
close all;
% ACADO FLAG
EXPORT = 1;
COMPILE = 1;
% State and control define
DifferentialState ee_px ee_py ee_pz quat_ex quat_ey quat_ez;
DifferentialState p1_px p1_py p1_pz p2_px p2_py p2_pz;
DifferentialState p3_px p3_py p3_pz p4_px p4_py p4_pz;
DifferentialState p5_px p5_py p5_pz p6_px p6_py p6_pz;
DifferentialState obj1_px obj1_py obj1_pz obj1_vx obj1_vy obj1_vz;
% input
Control joint1_v joint2_v joint3_v joint4_v joint5_v joint6_v;
% Online Data 18+18+54=90
OnlineData jp1_11 jp1_12;
OnlineData jp1_21 jp1_22;
OnlineData jp1_31 jp1_32;
OnlineData jp2_11 jp2_12;
OnlineData jp2_21 jp2_22;
OnlineData jp2_31 jp2_32;
OnlineData jp3_11 jp3_12;
OnlineData jp3_21 jp3_22;
OnlineData jp3_31 jp3_32;
OnlineData jp4_11 jp4_12 jp4_13;
OnlineData jp4_21 jp4_22 jp4_23;
OnlineData jp4_31 jp4_32 jp4_33;
OnlineData jp5_11 jp5_12 jp5_13;
OnlineData jp5_21 jp5_22 jp5_23;
OnlineData jp5_31 jp5_32 jp5_33;
OnlineData jp6_11 jp6_12 jp6_13 jp6_14 jp6_15 jp6_16;
OnlineData jp6_21 jp6_22 jp6_23 jp6_24 jp6_25 jp6_26;
OnlineData jp6_31 jp6_32 jp6_33 jp6_34 jp6_35 jp6_36;
OnlineData jee_11 jee_12 jee_13 jee_14 jee_15 jee_16;
OnlineData jee_21 jee_22 jee_23 jee_24 jee_25 jee_26;
OnlineData jee_31 jee_32 jee_33 jee_34 jee_35 jee_36;
OnlineData jee_41 jee_42 jee_43 jee_44 jee_45 jee_46;
OnlineData jee_51 jee_52 jee_53 jee_54 jee_55 jee_56;
OnlineData jee_61 jee_62 jee_63 jee_64 jee_65 jee_66;
h = [diffStates; controls];
hN = [diffStates];
% Differential Equation
f = acado.DifferentialEquation();
% ee
f.add(dot(ee_px) == jee_11*joint1_v+ jee_12*joint2_v+ jee_13*joint3_v+ jee_14*joint4_v+ jee_15*joint5_v+ jee_16*joint6_v);
f.add(dot(ee_py) == jee_21*joint1_v+ jee_22*joint2_v+ jee_23*joint3_v+ jee_24*joint4_v+ jee_25*joint5_v+ jee_26*joint6_v);
f.add(dot(ee_pz) == jee_31*joint1_v+ jee_32*joint2_v+ jee_33*joint3_v+ jee_34*joint4_v+ jee_35*joint5_v+ jee_36*joint6_v);
f.add(dot(quat_ex) == jee_41*joint1_v+ jee_42*joint2_v+ jee_43*joint3_v+ jee_44*joint4_v+ jee_45*joint5_v+ jee_46*joint6_v);
f.add(dot(quat_ey) == jee_51*joint1_v+ jee_52*joint2_v+ jee_53*joint3_v+ jee_54*joint4_v+ jee_55*joint5_v+ jee_56*joint6_v);
f.add(dot(quat_ez) == jee_61*joint1_v+ jee_62*joint2_v+ jee_63*joint3_v+ jee_64*joint4_v+ jee_65*joint5_v+ jee_66*joint6_v);
% p1
f.add(dot(p1_px) == jp1_11*joint1_v+ jp1_12*joint2_v);
f.add(dot(p1_py) == jp1_21*joint1_v+ jp1_22*joint2_v);
f.add(dot(p1_pz) == jp1_31*joint1_v+ jp1_32*joint2_v);
% p2
f.add(dot(p2_px) == jp2_11*joint1_v+ jp2_12*joint2_v);
f.add(dot(p2_py) == jp2_21*joint1_v+ jp2_22*joint2_v);
f.add(dot(p2_pz) == jp2_31*joint1_v+ jp2_32*joint2_v);
% p3
f.add(dot(p3_px) == jp3_11*joint1_v+ jp3_12*joint2_v);
f.add(dot(p3_py) == jp3_21*joint1_v+ jp3_22*joint2_v);
f.add(dot(p3_pz) == jp3_31*joint1_v+ jp3_32*joint2_v);
% p4
f.add(dot(p4_px) == jp4_11*joint1_v+ jp4_12*joint2_v+ jp4_13*joint3_v);
f.add(dot(p4_py) == jp4_21*joint1_v+ jp4_22*joint2_v+ jp4_23*joint3_v);
f.add(dot(p4_pz) == jp4_31*joint1_v+ jp4_32*joint2_v+ jp4_33*joint3_v);
% p5
f.add(dot(p5_px) == jp5_11*joint1_v+ jp5_12*joint2_v+ jp5_13*joint3_v);
f.add(dot(p5_py) == jp5_21*joint1_v+ jp5_22*joint2_v+ jp5_23*joint3_v);
f.add(dot(p5_pz) == jp5_31*joint1_v+ jp5_32*joint2_v+ jp5_33*joint3_v);
% p6
f.add(dot(p6_px) == jp6_11*joint1_v+ jp6_12*joint2_v+ jp6_13*joint3_v+ jp6_14*joint4_v+ jp6_15*joint5_v+ jp6_16*joint6_v);
f.add(dot(p6_py) == jp6_21*joint1_v+ jp6_22*joint2_v+ jp6_23*joint3_v+ jp6_24*joint4_v+ jp6_25*joint5_v+ jp6_26*joint6_v);
f.add(dot(p6_pz) == jp6_31*joint1_v+ jp6_32*joint2_v+ jp6_33*joint3_v+ jp6_34*joint4_v+ jp6_35*joint5_v+ jp6_36*joint6_v);

f.add(dot(obj1_px) == obj1_vx);
f.add(dot(obj1_py) == obj1_vy);
f.add(dot(obj1_pz) == obj1_vz);
f.add(dot(obj1_vx) == 0);
f.add(dot(obj1_vy) == 0);
f.add(dot(obj1_vz) == 0);
% Optimal Control Problem
acadoSet('problemname', 'mpc');
M = [0;0.04;0.2]; % control step is one 0.04s preidict horizon is 18 0.72s
ocp = acado.OCP(M);
W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));
ocp.minimizeLSQ(W, h);
ocp.minimizeLSQEndTerm(WN , hN);
% constraints
ocp.subjectTo( -0.6 <= joint1_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint2_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint3_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint4_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint5_v <= 0.6 );
ocp.subjectTo( -0.6 <= joint6_v <= 0.6 );
ocp.subjectTo( 0.25 <=power(power(p1_px-obj1_px,2)+power(p1_py-obj1_py,2)+power(p1_pz-obj1_pz,2),1/2) );
ocp.subjectTo( 0.25 <=power(power(p2_px-obj1_px,2)+power(p2_py-obj1_py,2)+power(p2_pz-obj1_pz,2),1/2) );
ocp.subjectTo( 0.25 <=power(power(p3_px-obj1_px,2)+power(p3_py-obj1_py,2)+power(p3_pz-obj1_pz,2),1/2) );
ocp.subjectTo( 0.25 <=power(power(p4_px-obj1_px,2)+power(p4_py-obj1_py,2)+power(p4_pz-obj1_pz,2),1/2) );
ocp.subjectTo( 0.25 <=power(power(p5_px-obj1_px,2)+power(p5_py-obj1_py,2)+power(p5_pz-obj1_pz,2),1/2) );
ocp.subjectTo( 0.25 <=power(power(p6_px-obj1_px,2)+power(p6_py-obj1_py,2)+power(p6_pz-obj1_pz,2),1/2) );
ocp.subjectTo( 0.25 <=power(power(ee_px-obj1_px,2)+power(ee_py-obj1_py,2)+power(ee_pz-obj1_pz,2),1/2) );


ocp.setModel(f);
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        1                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );
mpc.set( 'LEVENBERG_MARQUARDT',         1e-4                );
if EXPORT
    mpc.exportCode( 'export_MPC' );
end
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3') 
    cd export_MPC
    make_acado_solver_sfunction
    copyfile('acado_solver_sfun.mex*', '../')
    copyfile('acado_solver_sfun.tlc','../')
    cd ..
end
