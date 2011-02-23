/*!=============================================================================
  ==============================================================================

  \file    SL_kinematics_body.h

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  all kinematics functions.

  Note: this file was converted to a header file as it requires to include
        robot specific header files, such that it cannot be added to a SL
	library. SL_kinematics.c is just an empty file which includes
        this header.

  ============================================================================*/

// the system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "mdefs.h"
#include "SL_kinematics.h"
#include "utility_macros.h"

/* global variables */

/* local variables */

/* global functions */

/* local functions */

/* external variables */

/*!*****************************************************************************
 *******************************************************************************
\note  init_kinematics
\date  June 1999
   
\remarks 

        initializes the kinematics variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
init_kinematics(void)

{
  int i;

  link_pos             = my_matrix(0,N_LINKS,1,3);
  link_pos_des         = my_matrix(0,N_LINKS,1,3);
  link_pos_sim         = my_matrix(0,N_LINKS,1,3);
  joint_cog_mpos       = my_matrix(0,N_DOFS,1,3);
  joint_cog_mpos_des   = my_matrix(0,N_DOFS,1,3);
  joint_cog_mpos_sim   = my_matrix(0,N_DOFS,1,3);
  joint_origin_pos     = my_matrix(0,N_DOFS,1,3);
  joint_origin_pos_des = my_matrix(0,N_DOFS,1,3);
  joint_origin_pos_sim = my_matrix(0,N_DOFS,1,3);
  joint_axis_pos       = my_matrix(0,N_DOFS,1,3);
  joint_axis_pos_des   = my_matrix(0,N_DOFS,1,3);
  joint_axis_pos_sim   = my_matrix(0,N_DOFS,1,3);
 
  J                = my_matrix(1,N_ENDEFFS*6,1,N_DOFS);
  dJdt             = my_matrix(1,N_ENDEFFS*6,1,N_DOFS);
  Jdes             = my_matrix(1,N_ENDEFFS*6,1,N_DOFS);
  Jcog             = my_matrix(1,2*N_CART,1,N_DOFS);
  Jcogdes          = my_matrix(1,2*N_CART,1,N_DOFS);

  Jbase            = my_matrix(1,N_ENDEFFS*6,1,6);
  dJbasedt         = my_matrix(1,N_ENDEFFS*6,1,6);
  Jbasedes         = my_matrix(1,N_ENDEFFS*6,1,6);

  for (i=0; i<=N_LINKS; ++i) {
    Alink[i]     = my_matrix(1,4,1,4);
    Alink_des[i] = my_matrix(1,4,1,4);
    Alink_sim[i] = my_matrix(1,4,1,4);
  }

  // initialize indicators for prismatic joints
  for (i=0; i<=N_DOFS; ++i)
    prismatic_joint_flag[i] = FALSE;

#include "Prismatic_Joints.h"  

}

/*!*****************************************************************************
 *******************************************************************************
\note  linkInformation
\date  March 2005
   
\remarks 

        computes the m*cog, rotation axis, and local coord.sys. orgin for
        every link. This information can be used to compute the COG and
        COG jacobians, assuming the mass and center of mass parameters are
        properly identified.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
L
 \param[in]     state   : the state containing th, thd, thdd, and receiving the
                  appropriate u
 \param[in]     basec   : the position state of the base
 \param[in]     baseo   : the orientational state of the base
 \param[in]     endeff  : the endeffector parameters
 \param[out]    Xmcog   : array of mass*cog vectors 
 \param[out]    Xaxis   : array of rotation axes
 \param[out]    Xorigin : array of coord.sys. origin vectors
 \param[out]    Xlink   : array of link position
 \param[out]    Ahmat   : homogeneous transformation matrices of each link

 ******************************************************************************/
void 
linkInformation(SL_Jstate *state,SL_Cstate *basec,
		SL_quat *baseo, SL_endeff *eff, 
		double **Xmcog, double **Xaxis, double **Xorigin, double **Xlink,
		double ***Ahmat)

{

#include "LInfo_declare.h"

#include "LInfo_math.h"

}

/*!*****************************************************************************
 *******************************************************************************
\note  linkInformationDes
\date  March 2005
   
\remarks 

        computes the m*cog, rotation axis, and local coord.sys. orgin for
        every link. This information can be used to compute the COG and
        COG jacobians, assuming the mass and center of mass parameters are
        properly identified.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     state   : the state containing th, thd, thdd, and receiving the
                  appropriate u
 \param[in]     basec   : the position state of the base
 \param[in]     baseo   : the orientational state of the base
 \param[in]     endeff  : the endeffector parameters
 \param[out]    Xmcog   : array of mass*cog vectors 
 \param[out]    Xaxis   : array of rotation axes
 \param[out]    Xorigin : array of coord.sys. origin vectors
 \param[out]    Xlink   : array of link position
 \param[out]    Ahmat   : homogeneous transformation matrices of each link

 ******************************************************************************/
void 
linkInformationDes(SL_DJstate *state,SL_Cstate *basec,
		   SL_quat *baseo, SL_endeff *eff, 
		   double **Xmcog, double **Xaxis, double **Xorigin, 
		   double **Xlink, double ***Ahmat)

{

#include "LInfo_declare.h"
  
#include "LInfo_math.h"

}

/*!*****************************************************************************
 *******************************************************************************
\note  jacobian
\date  June 1999
   
\remarks 

        computes the jacobian

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lp      : the link positions
 \param[in]     jop     : joint origin positions
 \param[in]     jap     : joint axix unit vectors
 \param[out]    Jac     : the jacobian

 ******************************************************************************/
void 
jacobian(Matrix lp, Matrix jop, Matrix jap, Matrix Jac)

{
  int i,j,r;
  double c[2*N_CART+1];
#include "GJac_declare.h"

#include "GJac_math.h"

  /* the Jlist variable generated by the math files contains the 
     the indicators which joints contribute to each endeffector of
     the jacobian. Now, this information is used to compute the
     geometric jacobian */

  for (i=1; i<=n_endeffs; ++i) {
    for (j=1; j<=n_dofs; ++j) {
      if ( Jlist[i][j] != 0 ) {
	if (prismatic_joint_flag[j]) {
	  prismaticGJacColumn( lp[link2endeffmap[i]],
			      jop[j],
			      jap[j],
			      c );
	} else {
	  revoluteGJacColumn( lp[link2endeffmap[i]],
			      jop[j],
			      jap[j],
			      c );
	}
	for (r=1; r<=2*N_CART; ++r) 
	  Jac[(i-1)*6+r][j] = c[r];
      }
    }
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  baseJacobian
\date  June 1999
   
\remarks 

        computes the jacobian of the base coordinates w.r.t to all endeffectors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lp      : the link positions
 \param[in]     jop     : joint origin positions
 \param[in]     jap     : joint axix unit vectors
 \param[out]    Jb      : the jacobian

 ******************************************************************************/
void 
baseJacobian(Matrix lp, Matrix jop, Matrix jap, Matrix Jb)

{
  int i,j,r;
  double c[2*N_CART+1];
  
  // the base Jacobian is almost an identity matrix, except for the
  // upper right quadrant
  
  for (i=1; i<=n_endeffs; ++i) {
    for (r=1; r<=N_CART; ++r) {
      Jb[(i-1)*6+r][r]        = 1.0;
      Jb[(i-1)*6+r+3][r+3]    = 1.0;
    }
    for (j=1; j<=N_CART; ++j) {
      revoluteGJacColumn( lp[link2endeffmap[i]],
			  base_state.x,
			  Jb[j], // this is a equivalent to a unit vector
			  c );
      for (r=1; r<=N_CART; ++r) {
	Jb[(i-1)*6+r][N_CART+j] = c[r];
      }
    }
  }
  
}


/*!*****************************************************************************
 *******************************************************************************
\note  inverseKinematics
\date  Feb 2011
   
\remarks 

        computes the inverse kinematics based on the pseudo-inverse
        with optimization, using a robust SVD-based inversion

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state     : the state of the robot (given as a desired state)
 \param[in]     endeff  : the endeffector parameters
 \param[in]     rest    : the optimization posture
 \param[in]     cart    : the cartesian state (pos & orientations in a matrix)
 \param[in]     status  : which rows to use from the Jacobian
 \param[in]     dt      : the integration time step     

the function updates the state by adding the appropriate joint velocities
and by integrating the state forward with dt

The function returns the condition number of the inverted matrix. Normally,
condition number above 5000 become a bit critical. The SVD clips at a condition
number of 10000.

 ******************************************************************************/
double
inverseKinematics(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		  Vector cart, iVector status, double dt)
{
  
  int            i,j,n;
  int            count;
  static Matrix  Jac;
  static Matrix  P;
  static Matrix  B;
  static Matrix  O;
  static iVector ind;
  static Matrix  local_link_pos_des;
  static Matrix  local_joint_cog_mpos_des;
  static Matrix  local_joint_origin_pos_des;
  static Matrix  local_joint_axis_pos_des;
  static Matrix  local_Alink_des[N_LINKS+1];
  static int     firsttime = TRUE;
  double         ridge = 1.e-4;
  double         ralpha = 0.5;
  double         condnr;
  double         condnr_cutoff = 10000.0;

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    Jac  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
    P    = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    ind  = my_ivector(1,6*N_ENDEFFS);
    B    = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    O    = my_matrix(1,N_DOFS,1,N_DOFS);
    
    local_link_pos_des         = my_matrix(0,N_LINKS,1,3);
    local_joint_cog_mpos_des   = my_matrix(0,N_DOFS,1,3);
    local_joint_origin_pos_des = my_matrix(0,N_DOFS,1,3);
    local_joint_axis_pos_des   = my_matrix(0,N_DOFS,1,3);
    
    for (i=0; i<=N_LINKS; ++i)
      local_Alink_des[i] = my_matrix(1,4,1,4);
  }

  /* compute the Jacobian */
  linkInformationDes(state,&base_state,&base_orient,eff,
		     local_joint_cog_mpos_des,
		     local_joint_axis_pos_des,
		     local_joint_origin_pos_des,
		     local_link_pos_des,
		     local_Alink_des);

  jacobian(local_link_pos_des,local_joint_origin_pos_des,local_joint_axis_pos_des,Jac);

  /* how many contrained cartesian DOFs do we have? */
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  /* build the pseudo-inverse according to the status information */
  mat_zero(P);
  for (i=1; i<=count; ++i) {
    for (j=i; j<=count; ++j) {
      for (n=1; n<=N_DOFS; ++n) {
	P[i][j] += Jac[ind[i]][n] * Jac[ind[j]][n];
      }
      if (i==j) 
	P[i][j] += ridge;
      P[j][i] = P[i][j];
    }
  }

  // inversion with SVD with damping
  MY_MATRIX(M,1,count,1,count);
  MY_MATRIX(U,1,count,1,count);
  MY_MATRIX(V,1,count,1,count);
  MY_VECTOR(s,1,count);
  MY_VECTOR(si,1,count);
  mat_equal_size(P,count,count,U);
  my_svdcmp(U,count,count,s,V);

  // regularize if the condition number gets too large -- after the cutoff, we decay
  // the inverse of the singular value in a smooth way to zero
  for (i=1; i<=count; ++i)
    if (s[1]/(s[i]+1.e-10) > condnr_cutoff) {
      double sc = s[1]/condnr_cutoff;
      si[i] = -2./(sqr(sc)*sc)*sqr(s[i])+3./sqr(sc)*s[i];
    } else {
      si[i] = 1./s[i];
    }

  condnr = s[1]/(s[count]+1.e-10);

  // V*1/S*U' is the inverse
  for (i=1; i<=count; ++i) 
    for (j=1; j<=count; ++j)
      M[i][j] = U[j][i]*si[i];

  mat_mult(V,M,P);
  
  /* build the B matrix, i.e., the pseudo-inverse */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=count; ++j) {
      B[i][j]=0.0;
      for (n=1; n<=count; ++n) {
	B[i][j] += Jac[ind[n]][i] * P[n][j];
      }
    }
  }

  /* this provides the first part of the optimized pseudo inverse */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thd = 0;
    for (j=1; j<=count; ++j) {
      state[i].thd += B[i][j] * cart[ind[j]];
    }
  }

  /* the optimization part */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=i; j<=N_DOFS; ++j) {
      if (i==j) 
	O[i][j] = 1.0;
      else
	O[i][j] = 0.0;
      for (n=1; n<=count; ++n) {
	O[i][j] -= B[i][n] * Jac[ind[n]][j];
      }
      O[j][i] = O[i][j];
    }
  }

  /* add the optimization part to the velocities */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=N_DOFS; ++j) {
      state[i].thd += ralpha* rest[j].w * (rest[j].th - state[j].th) * O[i][j];
    }
  }

  /* integrate forward */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].th += state[i].thd * dt;
  }

  return condnr;

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeLinkVelocity
\date  March 2006
   
\remarks 

        Computes the velocity of a particular link in world coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lID    : the ID of the link
 \param[in]     lp     : the link positions
 \param[in]     jop    : joint origin positions
 \param[in]     jap    : joint axix unit vectors
 \param[in]     js     : joint state
 \param[out]    v      : velocity vector

 ******************************************************************************/
void 
computeLinkVelocity(int lID, Matrix lp, Matrix jop, Matrix jap, 
		    SL_Jstate *js, double *v)
{
  int i,j,r;
  double c[2*N_CART+1];
  MY_MATRIX(Jlink,1,N_CART,1,n_dofs);
  MY_MATRIX(Jlinkbase,1,N_CART,1,2*N_CART);

#include "Contact_GJac_declare.h"
#include "Contact_GJac_math.h"

  /* the Jlist variable generated by the math files contains the 
     the indicators which joints contribute to each link position
     in the jacobian. Now, this information is used to compute the
     geometric jacobian */
  
  for (j=1; j<=n_dofs; ++j) {
    if ( Jlist[lID][j] != 0 ) {
      if (prismatic_joint_flag[j]) {
	prismaticGJacColumn( lp[lID],
			    jop[j],
			    jap[j],
			    c );
      } else {
	revoluteGJacColumn( lp[lID],
			    jop[j],
			    jap[j],
			    c );
      }
      for (r=1; r<=N_CART; ++r) 
	Jlink[r][j] = c[r];
    }
  }

  // next the base Jacobian
  // the base Jacobian is almost an identity matrix, except for the
  // upper right quadrant
  
  for (r=1; r<=N_CART; ++r) {
    Jlinkbase[r][r]        = 1.0;
  }
  for (j=1; j<=N_CART; ++j) {
    revoluteGJacColumn( lp[lID],
			base_state.x,
			Jlinkbase[j], // this is a equivalent to a unit vector
			c );
    for (r=1; r<=N_CART; ++r) {
      Jlinkbase[r][N_CART+j] = c[r];
    }
  }

  // compute the link velocity from Jlink and the base jacobian
  for (i=1; i<=N_CART; ++i) {

    v[i]     = 0.0;

    /* contributations from the joints */
    for (r=1; r<=n_dofs; ++r) {
      v[i] += Jlink[i][r] * js[r].thd;
    }

    /* contributations from the base */
    for (r=1; r<=N_CART; ++r) {
      v[i] += Jlinkbase[i][r] * base_state.xd[r];
      v[i] += Jlinkbase[i][3+r] * base_orient.ad[r];
    }
    
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeConstraintJacobian
\date  July 2009
   
\remarks 

   from the constraint information in the endeff structure, the constraint
   Jacobian for the floating base dynamics is computed

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     state   : the joints state
 \param[in]     basec   : cartesin info of base
 \param[in]     baseo   : orientation info of base
 \param[in]     eff     : endeffector info
 \param[out]    Jc      : constraint Jacobian
 \param[out]    nr      : number of rows in Jc
 \param[out]    nc      : number of columns in Jc

     note: memory for Jc needs to be provided as N_ENDEFFS*6 x N_DOFS+6


 ******************************************************************************/
void 
computeConstraintJacobian(SL_Jstate *state,SL_Cstate *basec,
			  SL_quat *baseo, SL_endeff *eff, 
			  Matrix Jc, int *nr, int *nc)
{
  int i,j,m,n;
  int count;
  MY_MATRIX(Xmcog,0,N_DOFS,1,3);
  MY_MATRIX(Xaxis,0,N_DOFS,1,3);
  MY_MATRIX(Xorigin,0,N_DOFS,1,3);
  MY_MATRIX(Xlink,0,N_LINKS,1,3);
  MY_MATRIX_ARRAY(Ahmat,1,4,1,4,N_LINKS);
  MY_MATRIX(Ec,1,(N_CART*2),1,(N_CART*2));
  MY_MATRIX(Jac,1,(N_ENDEFFS*2*N_CART),1,N_DOFS);
  
  mat_eye(Ec);
  
  // compute the link information for this state
  linkInformation(state, basec, baseo, eff, 
		  Xmcog, Xaxis, Xorigin, Xlink, Ahmat);
  
  // compute Jacobian
  jacobian(Xlink, Xorigin, Xaxis, Jac);

  // create the Jc matrix
  count = 0;
  for (i=1; i<=N_ENDEFFS; ++i) {

    // compute the Jacobian component due to the base: these results
    // come from cross productcs of the unit vectors of the base coordinate
    // with the endeffector-base position.
    Ec[_Y_][N_CART+_X_] = -(Xlink[link2endeffmap[i]][_Z_] - basec->x[_Z_]);
    Ec[_Z_][N_CART+_X_] =   Xlink[link2endeffmap[i]][_Y_] - basec->x[_Y_];

    Ec[_X_][N_CART+_Y_] =   Xlink[link2endeffmap[i]][_Z_] - basec->x[_Z_];
    Ec[_Z_][N_CART+_Y_] = -(Xlink[link2endeffmap[i]][_X_] - basec->x[_X_]);

    Ec[_X_][N_CART+_Z_] = -(Xlink[link2endeffmap[i]][_Y_] - basec->x[_Y_]);
    Ec[_Y_][N_CART+_Z_] =   Xlink[link2endeffmap[i]][_X_] - basec->x[_X_];

    for (j=1; j<=2*N_CART; ++j) {

      if (eff[i].c[j]) {

	++count;

	for (m=1; m<=N_DOFS; ++m)
	  (Jc)[count][m] = Jac[(i-1)*2*N_CART+j][m];

	for (m=1; m<=2*N_CART; ++m)
	  (Jc)[count][N_DOFS+m] = Ec[j][m];	  

      }

    }
  }

  *nr = count;
  *nc = 2*N_CART+N_DOFS;

}
 

/*!*****************************************************************************
 *******************************************************************************
\note  computeQR
\date  Nov. 2008
\remarks 

          computes the QR decomposition of a Jacobian using SVD


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

               nf = N_DOFS+6 : full state dim
               nu = nf - nr  : unconstraint space dim

 \param[in]     Jc : the Jacobian
 \param[in]     nr : number of rows in Jacobian
 \param[in]     nc : number of columns in Jacobian
 \param[out]    Q : Q matrix (needs to be nf x nf)
 \param[out]    Qu : the part of Q that is the basis of unconstraint space (nf x nu)
 \param[out]    R : R matrix (needs to be at least nr x nr)
 \param[out]    sv : singular values (needs to be nf)

 ******************************************************************************/
void 
computeQR(Matrix Jc, int nr, int nc, Matrix Q, Matrix Qu, Matrix R, Vector sv)

{
  int i,j,n,m;
  int nf = N_DOFS+6;          // DOFs of full floating base space
  int nu = N_DOFS+6 - nr;     // number of unconstraint DOFs
  double aux;

  MY_MATRIX(U,1,nr,1,nf);
  MY_MATRIX(Su,1,nu,1,nf);

  // compute an orthonormal basis of the constraint Jacobian  with SVD
  mat_equal_size(Jc,nr,nf,U);
  my_svdcmp(U, nr, nf, sv, Q);

  // sort the columns of U and Q accoring to w coefficients
  for (j=1; j<=nf; ++j)
    for (i=1; i<=nf-j; ++i)
      if (sv[i] < sv[i+1]) {
	for (n=1; n<=nf; ++n) {
	  aux = Q[n][i];
	  Q[n][i] = Q[n][i+1];
	  Q[n][i+1] = aux;
	}
	for (n=1; n<=nr; ++n) {
	  aux = U[n][i];
	  U[n][i] = U[n][i+1];
	  U[n][i+1] = aux;
	}
	aux = sv[i];
	sv[i]=sv[i+1];
	sv[i+1]=aux;
      }

  // determine Su (nu x nf)
  mat_zero(Su);
  for (i=1; i<=nu; ++i)
    Su[i][nr+i] = 1.0;

  // Qu = Q*SuT  (nf x nu)
  mat_mult_normal_transpose(Q,Su,Qu);

  // this is the R matrix from the QR decomposition, derived from SVD, i.e.,
  // J = U S V'; J' = Q R; thus J' = V S U', and V = Q, S U' = R
  for (i=1; i<=nr; ++i)
    for (j=1; j<=nr; ++j)
      R[i][j] = U[j][i]*sv[i];

}

