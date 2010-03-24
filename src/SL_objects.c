/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_objects.c

  \author 
  \date   

  ==============================================================================
  \remarks

  handling of objects in SL

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_objects.h"
#include "SL_terrains.h"
#include "SL_common.h"
#include "SL_kinematics.h"
#include "SL_simulation_servo.h"
#include "SL_shared_memory.h"

// global variables
ObjectPtr  objs = NULL;
ContactPtr contacts=NULL;
SL_uext   *ucontact;

// local variables
static int opengl_update_rate = 60;   // 60Hz should be enough for updating the openGl screen
static int opengl_update_counter = 0; // Counter to keep track when openGL screen should be updated

// global functions 

// local functions
static ObjectPtr addObject(char *name, int type, int contact, 
			   double *rgb, double *trans, double *rot, 
			   double *scale, Vector cspecs, Vector ospecs);
static void  computeContactForces(ObjectPtr optr, ContactPtr cptr, int ind);
static void  contactVelocity(int cID, ObjectPtr optr, double *v);

// external functions

/*!*****************************************************************************
 *******************************************************************************
\note  initObjects
\date  Nov 2007
\remarks 

        initializes object related variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
initObjects(void) 
{

  // contacts
  contacts = my_calloc(n_links+1,sizeof(Contact),MY_STOP);
  ucontact = my_calloc(n_dofs+1,sizeof(SL_uext),MY_STOP);

  return TRUE;

}
 
/*!*****************************************************************************
 *******************************************************************************
\note  addObject
\date  Nov 2000
   
\remarks 

       add an object to the environment

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     type       : object type
 \param[in]     contact    : contact type
 \param[in]     rgb        : rgb values for color
 \param[in]     trans      : translatory offset
 \param[in]     rot        : rotary offset
 \param[in]     scale      : scaling of object
 \param[in]     cspecs     : contact specifications
 \param[in]     ospecs     : object specifications
 \param[in]     n_ops      : number of object parameters
 \param[in]     n_cps      : number of contact pararmeters

 ******************************************************************************/
static ObjectPtr
addObject(char *name, int type, int contact, double *rgb, double *trans, double *rot, 
	  double *scale, Vector cspecs, Vector ospecs)

{
  int        i;
  ObjectPtr  ptr=NULL;
  ObjectPtr  last_ptr=NULL;
  char       string[1000];
  int        n_cps=0, n_ops=0;
  int        new_obj_flag = TRUE;

  /* is object already present and only needs update? */
  if (objs != NULL) {

    ptr = objs;
    do {
      if (strcmp(ptr->name,name)==0) {
	new_obj_flag = FALSE;
	break;
      }
      last_ptr = ptr;
      ptr = (ObjectPtr) ptr->nptr;
    } while ( ptr != NULL);
    
  }

  if (new_obj_flag) {
    /* allocate new object */
    ptr = my_calloc(1,sizeof(Object),MY_STOP);
  }

  if (cspecs == NULL) {
    n_cps = 0;
  } else {
    n_cps = cspecs[0];
    if (n_cps > 0 && new_obj_flag)
      ptr->contact_parms = my_vector(1,n_cps);
  }
  if (ospecs == NULL) {
    n_ops = 0;
  } else {
    n_ops = ospecs[0];
    if (n_ops > 0 && new_obj_flag)
      ptr->object_parms  = my_vector(1,n_ops);
  }

  /* assign values */
  strcpy(ptr->name, name);
  ptr->type = type;
  ptr->display_list_active = FALSE;
  ptr->hide = FALSE;
  ptr->contact_model = contact;
  for (i=1; i<=N_CART; ++i) {
    ptr->rgb[i]   = rgb[i];
    ptr->trans[i] = trans[i];
    ptr->rot[i]   = rot[i];
    ptr->scale[i] = scale[i];
  }
  for (i=1; i<=n_ops; ++i) {
    ptr->object_parms[i]=ospecs[i];
  }
  for (i=1; i<=n_cps; ++i) {
    ptr->contact_parms[i]=cspecs[i];
  }

  if (new_obj_flag) {
    ptr->nptr=NULL;
    if (objs == NULL) {
      objs = ptr;
    } else {
      last_ptr->nptr = (char *) ptr;
    }
  }

  return ptr;

}

/*!*****************************************************************************
 *******************************************************************************
\note  addCube
\date  Nov 2000
   
\remarks 

       add a cube to the object list

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     rgb        : rgb values for color
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector
 \param[in]     scale      : pointer to rotation vector
 \param[in]     contact    : ID of contact model  
 \param[in]     parms      : array of contact parameters

 ******************************************************************************/
ObjectPtr
addCube(char *name, double *rgb, double *pos, double *rot, 
	double *scale, int contact,
	double *parms)

{
  return (addObject(name,CUBE,contact,rgb,pos,rot,scale,parms,NULL));
}

/*!*****************************************************************************
 *******************************************************************************
\note  addSphere
\date  Nov 2000
   
\remarks 

       add a sphere to the object list

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     rgb        : rgb values for color
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector
 \param[in]     scale      : pointer to rotation vector
 \param[in]     contact    : ID of contact model  
 \param[in]     parms      : array of contact parameters
 \param[in]     n_faces    : number of faces


 ******************************************************************************/
ObjectPtr
addSphere(char *name, double *rgb, double *pos, double *rot, 
	  double *scale, int contact,
	  Vector parms, int n_faces)
{
  double ospecs[1+1];

  ospecs[0]=1;
  ospecs[1]=n_faces;

  return (addObject(name,SPHERE,contact,rgb,pos,rot,scale,parms,ospecs));

}

/*!*****************************************************************************
 *******************************************************************************
\note  changeObjPosByName
\date  Nov 2000
   
\remarks 

       changes the object position by name (slower, since search is required)

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector

 ******************************************************************************/
int
changeObjPosByName(char *name, double *pos, double *rot)
{
  ObjectPtr ptr = getObjPtrByName(name);
  if (ptr == NULL) 
    return FALSE;

  changeObjPosByPtr(ptr, pos, rot);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  changeHideObjByName
\date  Nov 2000
   
\remarks 

       changes the hide status of object  by name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     hide       : TRUE/FALSE

 ******************************************************************************/
int
changeHideObjByName(char *name, int hide)
{
  ObjectPtr ptr;
  int i;

  if (objs == NULL) 
    return FALSE;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      ptr->hide = hide;
      return TRUE;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  getObjForcesByName
\date  June 2007
   
\remarks 

   returns the force and torque vector acting on an object

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     f          : pointer to force vector 
 \param[in]     t          : pointer to torque vector

 ******************************************************************************/
int
getObjForcesByName(char *name, double *f, double *t)
{
  ObjectPtr ptr;
  int i;

  if (objs == NULL) 
    return FALSE;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      for (i=1;i<=N_CART; ++i) {
	f[i] = ptr->f[i];
	t[i] = ptr->t[i];
      }
      return TRUE;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  getObjForcesByPtr
\date  June 2007
   
\remarks 

   returns the force and torque vector acting on an object

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ptr        : pointer to object
 \param[in]     f          : pointer to force vector 
 \param[in]     t          : pointer to torque vector

 ******************************************************************************/
int
getObjForcesByPtr(ObjectPtr ptr, double *f, double *t)
{
  int i;

  for (i=1;i<=N_CART; ++i) {
    f[i] = ptr->f[i];
    t[i] = ptr->t[i];
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  getObjPtrByName
\date  Nov 2000
   
\remarks 

   returns the pointer to a specific object

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 

 ******************************************************************************/
ObjectPtr
getObjPtrByName(char *name)
{
  ObjectPtr ptr;
  int i;

  if (objs == NULL) 
    return NULL;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      return ptr;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return NULL;
  
}

	

/*!*****************************************************************************
 *******************************************************************************
\note  changeObjPosByPtr
\date  Nov 2000
   
\remarks 

       changes the object position by pointer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ptr        : ptr of the object 
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector

 ******************************************************************************/
void
changeObjPosByPtr(ObjectPtr ptr, double *pos, double *rot)
{

  int i;
  
  for (i=1;i<=N_CART; ++i) {
    ptr->trans[i]=pos[i];
    ptr->rot[i]=rot[i];
  }
  
  if (strcmp(servo_name,"task")==0) { // communicate info to other servos
    struct {
      char   obj_name[100];
      double pos[N_CART+1];
      double rot[N_CART+1];
    } data;
    unsigned char buf[sizeof(data)];
    
    strcpy(data.obj_name,ptr->name);
    for (i=1;i<=N_CART; ++i) {
      data.pos[i]=pos[i];
      data.rot[i]=rot[i];
    }
    
    memcpy(buf,&data,sizeof(data));
    
    // Send the data to the simulation server.
    sendMessageSimulationServo("changeObjPosByName",(void *)buf,sizeof(data));
    
    // Freek:
    // The opengl server does not need to be updated so often.
    // So what we do is keep a counter that is reset everytime it reaches the value
    // (servo_base_rate/opengl_update_rate), i.e. if servo_base_rate=1000Hz and
    // opengl_update_rate=50Hz, we update the opengl server every 20th time.
    if ((opengl_update_counter++)>=(servo_base_rate/opengl_update_rate)) {
      sendMessageOpenGLServo("changeObjPosByName",(void *)buf,sizeof(data));
      opengl_update_counter = 0;
    }
    // Some debug info:
    //printf("Update rates: base=%dHz opengl=%dHz",servo_base_rate,opengl_update_rate);
    //printf("Count=%d/%d\n",opengl_update_counter,(servo_base_rate/opengl_update_rate));
	}
}

/*!*****************************************************************************
 *******************************************************************************
\note  deleteObjByName
\date  Nov 2000
   
\remarks 

       deletes an object by name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 

 ******************************************************************************/
int
deleteObjByName(char *name)
{
  ObjectPtr ptr,ptr2;
  char     *nptr;
  int i;

  if (objs == NULL) 
    return FALSE;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      ptr2=objs;
      if (ptr2 == ptr) {
	objs = (ObjectPtr) ptr->nptr;
      } else {
	do {
	  if (ptr2->nptr == (char *)ptr) {
	    ptr2->nptr = ptr->nptr;
	    break;
	  }
	  ptr2 = (ObjectPtr) ptr2->nptr;
	} while (ptr2 != NULL);
      }
      free(ptr);
      return TRUE;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  checkContacts
\date  June 1999
   
\remarks 

      checks for contacts and computes the contact forces

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
checkContacts(void)

{
  int       i,j;
  ObjectPtr optr;
  double    x[N_CART+1];
  double    aux,aux1,aux2;
  int       ind=0;
  int       first_contact_flag = FALSE;
  int       contact_flag = FALSE;
  double    z;
  double    n[N_CART+1];
  double    v[N_CART+1];
  char      tfname[100];
  double    no_go;

  /* zero contact forces */
  bzero((void *)ucontact,sizeof(SL_uext)*(n_dofs+1));

  /* if there are no objects, exit */
  if (objs==NULL)
    return;

  // zero all object forces and torques
  optr = objs;
  do { 
    for (i=1; i<=N_CART; ++i)
      optr->f[i] = optr->t[i] = 0.0;
    optr = (ObjectPtr) optr->nptr;
  } while (optr != NULL);


  for (i=0; i<=n_links; ++i) { /* loop over all contact points */

    if (!contacts[i].active)
      continue;

    first_contact_flag = FALSE;
    contact_flag = FALSE;

    optr = objs;
    do {   /* check all objects */

      /* check whether this is a contact */
      if (optr->contact_model == NO_CONTACT) {
	optr = (ObjectPtr) optr->nptr;
	continue;
      }

      /* step one: transform potential contact point into object
	 coordinates */

      // the link point relative to object
      for (j=1; j<=N_CART; ++j)
	x[j] = link_pos_sim[i][j] - optr->trans[j];
      
      // the link point in object centered coordinates
      if (optr->rot[1] != 0.0) {
	aux  =  x[2]*cos(optr->rot[1])+x[3]*sin(optr->rot[1]);
	x[3] = -x[2]*sin(optr->rot[1])+x[3]*cos(optr->rot[1]);
	x[2] = aux;
      }

      if (optr->rot[2] != 0.0) {
	aux  =  x[1]*cos(optr->rot[2])-x[3]*sin(optr->rot[2]);
	x[3] =  x[1]*sin(optr->rot[2])+x[3]*cos(optr->rot[2]);
	x[1] = aux;
      }

      if (optr->rot[3] != 0.0) {
	aux  =  x[1]*cos(optr->rot[3])+x[2]*sin(optr->rot[3]);
	x[2] = -x[1]*sin(optr->rot[3])+x[2]*cos(optr->rot[3]);
	x[1] = aux;
      }

      // is this point inside the object? 
      switch (optr->type) {
      case CUBE: //---------------------------------------------------------------
	if (fabs(x[1]) < optr->scale[1]/2. &&
	    fabs(x[2]) < optr->scale[2]/2. &&
	    fabs(x[3]) < optr->scale[3]/2.) {

	  // remember which object we are contacting, and also the 
	  // contact point in object centered coordinates

	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }

	  // compute relevant geometric information

	  if (first_contact_flag) {
	    // what is the closest face of the cube
	    if ( (optr->scale[1]/2.-fabs(x[1]) ) < (optr->scale[2]/2.-fabs(x[2]) ) &&
		 (optr->scale[1]/2.-fabs(x[1]) ) < (optr->scale[3]/2.-fabs(x[3]) )) {
	      ind = 1;
	    } else if ( (optr->scale[2]/2.-fabs(x[2]) ) < (optr->scale[1]/2.-fabs(x[1]) ) &&
			(optr->scale[2]/2.-fabs(x[2]) ) < (optr->scale[3]/2.-fabs(x[3]) )) {
	      ind = 2;
	    } else {
	      ind = 3;
	    }
	    contacts[i].face_index = ind;
	  } else {
	    ind = contacts[i].face_index;
	  }

	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);
	  
	  // the normal vector
	  for (j=1; j<=N_CART; ++j) {
	    if (j==ind) {
	      // the contact normal never change direction relativ to the start point
	      contacts[i].normal[j] = 
		optr->scale[j]/2.*macro_sign(contacts[i].x_start[j]) - x[j];
	      contacts[i].normvel[j] = -v[j];
	    } else {
	      contacts[i].normal[j] = 0.0;
	      contacts[i].normvel[j] = 0.0;
	    }
	  }

	  // the tangential vector
	  for (j=1; j<=N_CART; ++j) {
	    if (j!=ind) {
	      contacts[i].tangent[j] = x[j]-contacts[i].x_start[j];
	      contacts[i].tanvel[j]  = v[j];
	    } else {
	      contacts[i].tangent[j] = 0.0;
	      contacts[i].tanvel[j]  = 0.0;
	    }
	  }

	  // the tangential velocity for viscous friction
	  for (j=1; j<=N_CART; ++j) {
	    if (j!=ind) {
	      contacts[i].viscvel[j] = v[j];
	    } else {
	      contacts[i].viscvel[j]=0.0;
	    }
	  }

	  computeContactForces(optr,&contacts[i],i);
	  optr = NULL;
	  continue; /* only one object can be in contact with a contact point */

	}

	break;

      case SPHERE: //---------------------------------------------------------------
	if ((sqr(x[1]/optr->scale[1]*2.) + sqr(x[2]/optr->scale[2]*2.) + 
	     sqr(x[3]/optr->scale[3]*2.)) < 1.0) {

	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }

	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);

	  /* the normal displacement vector: map the current x into the unit sphere,
	     compute the point where this vector pierces through the unit sphere,
	     and map it back to the deformed sphere. The difference between this
	     vector and x is the normal displacement */
	  aux2 = 1.e-10;
	  for (j=1; j<=N_CART; ++j) {
	    aux2 += sqr(x[j]/(optr->scale[j]/2.));
	  }
	  aux2 = sqrt(aux2); /* length in unit sphere */

	  aux = 1.e-10;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].normal[j] = x[j]*(1./aux2-1.);
	    aux += sqr(contacts[i].normal[j]);
	  }
	  aux = sqrt(aux);

	  // unit vector of contact normal
	  aux2 = 0.0;
	  for (j=1; j<=N_CART; ++j) {
	    n[j] = contacts[i].normal[j]/aux;
	    aux2 += n[j]*v[j];
	  }

	  // the normal velocity
	  for (j=1; j<=N_CART; ++j)
	    contacts[i].normvel[j] = -aux2*n[j];

	  /* the tangential vector */
	  aux1 = 0.0;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j]= x[j]-contacts[i].x_start[j];
	    aux1 += n[j] * contacts[i].tangent[j];
	  }
	  
	  // subtract the all components in the direction of the normal
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j] -= n[j] * aux1;
	    contacts[i].tanvel[j]   = v[j] - n[j]*aux2;
	  }

	  /* the vicous velocity */
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].viscvel[j] = v[j] - n[j]*aux2;
	  }

	  computeContactForces(optr,&contacts[i],i);
	  optr=NULL;
	  continue; /* only one object can be in contact with a contact point */

	} 

	break;

	
      case TERRAIN: //---------------------------------------------------------------
	sprintf(tfname,"%s.asc",optr->name); 
	if (!getContactTerrainInfo(x[1], x[2], tfname, &z, n, &no_go))
	  break;

	if (x[3] < z) {
	
	  // remember which object we are contacting, and also the 
	  // contact point in object centered coordinates
	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }

	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);
	  aux1 = n[_X_]*v[_X_]+n[_Y_]*v[_Y_]+n[_Z_]*v[_Z_];
	  
	  // compute relevant geometric information

	  // the normal vector
	  for (j=1; j<=N_CART; ++j) {
	    // note: n'*[ 0 0 (z-x[3])] = (z-x[3])*n[3] is the effective
	    // projection of the vertical distance to the surface onto
	    // the normal
	    contacts[i].normal[j]   = n[j]*((z-x[3])*n[3]);
	    contacts[i].normvel[j]  = -aux1*n[j];
	  }

	  // the tangential vector: project x-x_start into the null-space of normal
	  aux  = 0.0;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j]=x[j]-contacts[i].x_start[j];
	    aux  += contacts[i].tangent[j]*n[j];
	  }

	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j] -= n[j]*aux;
	    contacts[i].tanvel[j]   = v[j]-n[j]*aux1;
	  }

	  // the tangential velocity for viscous friction
	  for (j=1; j<=N_CART; ++j)
	    contacts[i].viscvel[j] = v[j]-n[j]*aux1;

	  computeContactForces(optr,&contacts[i],i);
	  optr = NULL;
	  continue; /* only one object can be in contact with a contact point */

	}

	break;

      }

      optr = (ObjectPtr) optr->nptr;

    } while (optr != NULL);

    contacts[i].status = contact_flag;

    if (!contacts[i].status) { // this is just for easy data interpretation
      for (j=1; j<=N_CART; ++j) {
	contacts[i].normal[j] = 0.0;
	contacts[i].normvel[j] = 0.0;
	contacts[i].tangent[j] = 0.0;
	contacts[i].tanvel[j] = 0.0;
	contacts[i].viscvel[j] = 0.0;
	contacts[i].f[j] = 0.0;
      }
    }

  }

  /* add simulated external forces to contact forces */
  for (i=0; i<=n_dofs; ++i)
    for (j=_X_; j<=_Z_; ++j) {
      ucontact[i].f[j] += uext_sim[i].f[j];
      ucontact[i].t[j] += uext_sim[i].t[j];
    }

}

/*!*****************************************************************************
 *******************************************************************************
\note  readObjects
\date  June 2006
\remarks 

Parses an arbitrary objects file. Objects are defined by multiple lines, given 
as in the following example:

floor 1 		( name and ID of object type )
1.0 0.0 0.0          	( rgb color specification)
0.0 0.0 -1.65  		( 3D position of center of object )
0.0 0.0 0.0  		( x-y-z Euler angle orientation )
40.0 40.0 1.0  		( scale applied to x, y, z of object)
2  			( contact model: see computeContactForces() below )
10         		( object parameters: see SL_openGL.c: drawObjects() )
1000 30 1000 30 0.5 0.4	( contact parameters: see computeContactForces() below )

The number of object parameters and contact parameters can vary for
different objects and different contact models. The parsing of these
parameters is pretty rudimentary, and simply all lines are expected in
the correct sequence, and the "new-line" character is interpreted as the
end of line indicator.


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     cfname : file name of objects file

 ******************************************************************************/
void
readObjects(char *cfname) 

{
  int j, i, ID;
  FILE  *in;
  double dum;
  double rgb[N_CART+1];
  double pos[N_CART+1];
  double rot[N_CART+1];
  double scale[N_CART+1];
  int    contact;
  int    objtype;
  char   name[100];
  char   fname[100];
  char   name2[100];
  double oparms[30+1];
  double cparms[30+1];
  double display_grid_delta;
  ObjectPtr optr;
  double x,y,z;
  double n[N_CART+1];
  char   string[100];

  
  sprintf(string,"%s%s",CONFIG,cfname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return;
  }

  /* find objects in file */
  while (fscanf(in,"%s %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d",
		name,&objtype,&rgb[1],&rgb[2],&rgb[3],
		&pos[1],&pos[2],&pos[3],&rot[1],&rot[2],&rot[3],
		&scale[1],&scale[2],&scale[3],&contact) == 15) {

    while (fgetc(in) != '\n')
      ;
    i=0;
    while ((string[i++]=fgetc(in))!= '\n' && i<499 )
      ;
    string[i]='\0';
    i=sscanf(string,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	     &oparms[1],&oparms[2],&oparms[3],&oparms[4],&oparms[5],
      	     &oparms[6],&oparms[7],&oparms[8],&oparms[9],&oparms[10],
	     &oparms[11],&oparms[12],&oparms[13],&oparms[14],&oparms[15],
	     &oparms[16],&oparms[17],&oparms[18],&oparms[19],&oparms[20],
	     &oparms[21],&oparms[22],&oparms[23],&oparms[24],&oparms[25],
	     &oparms[26],&oparms[27],&oparms[28],&oparms[29],&oparms[30]);
    oparms[0] = i;

    i=0;
    while ((string[i++]=fgetc(in)) != '\n' && i<499)
      ;
    string[i]='\0';
    i=sscanf(string,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	     &cparms[1],&cparms[2],&cparms[3],&cparms[4],&cparms[5],
      	     &cparms[6],&cparms[7],&cparms[8],&cparms[9],&cparms[10],
	     &cparms[11],&cparms[12],&cparms[13],&cparms[14],&cparms[15],
	     &cparms[16],&cparms[17],&cparms[18],&cparms[19],&cparms[20],
	     &cparms[21],&cparms[22],&cparms[23],&cparms[24],&cparms[25],
	     &cparms[26],&cparms[27],&cparms[28],&cparms[29],&cparms[30]);
    cparms[0] = i;

    optr = addObject(name,objtype,contact,rgb,pos,rot,scale,cparms,oparms);
    
    /* if there is a floor, initialize the gound level for the terrains */
    if (strcmp("floor",name) == 0) {
      setTerrainGroundZ(optr->trans[_Z_] + 0.5*optr->scale[_Z_]);
      printf("\nFound object >floor< and intialized ground level to %f\n",
	     optr->trans[_Z_] + 0.5*optr->scale[_Z_]);
    }
    
    // for terrains we need to add the terrain and create a display list for this terrain
    if (objtype == TERRAIN) {
      sprintf(fname,"%s.asc",name); 
      if ((ID=getNextTerrainID())) { 
	SL_quat q;
	
	eulerToQuat(rot,&q);
	// Note: oparms[1] = reg_rad oparms[2]=reg_down oparms[3]=reg_crad oparms[4]=disp_grid_delta
	setTerrainInfo(ID,ID,fname,pos,q.q,
		       (int)rint(oparms[1]),(int)rint(oparms[2]),(int)rint(oparms[3]));
	
      } else {
	printf("No more terrains possible -- increase MAX_TERRAINS\n");
      }
    }
    
  }
  
  fclose(in);
  remove_temp_file();

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeContactForces
\date  Dec.2000
   
\remarks 

      for a given contact point and object, compute the contact forces
      and add them to the appropriate structure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     optr  : ptr to object
 \param[in]     ctpr  : ptr to contact point
 \param[in]     ind   : index of the current contact point

 ******************************************************************************/
static void
computeContactForces(ObjectPtr optr, ContactPtr cptr, int ind)

{
  int i,j;
  double aux;
  double temp[N_CART+1];
  double temp1[N_CART+1];
  double temp2[N_CART+1];
  double moment_arm[N_CART+1];
  double moment_arm_object[N_CART+1];
  double normal_force;
  double normal_velocity;
  double tangent_force;
  double tangent_velocity;
  double viscvel;

  /* compute the contact forces in object centered coordinates */
  switch (optr->contact_model) {

  case DAMPED_SPRING_STATIC_FRICTION:
    
    /* note: the contact_parms arrays has the following elements:
       contact_parms[1] = normal spring coefficient
       contact_parms[2] = normal damping coefficient
       contact_parms[3] = static friction spring coefficient
       contact_parms[4] = static friction damping spring coefficient
       contact_parms[5] = static friction coefficient (friction cone)
       contact_parms[6] = dynamic friction coefficient (proportional to normal force)
    */
    
    // the normal contact force in object centered coordinates
    normal_force = 0;
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] = optr->contact_parms[1] * cptr->normal[i] +
	optr->contact_parms[2] * cptr->normvel[i];
      // make sure the damping part does not attract a contact force with wrong sign
      if (macro_sign(cptr->f[i])*macro_sign(cptr->normal[i])<0)
	cptr->f[i] = 0.0;
      
      normal_force += sqr(cptr->f[i]);
    }
    normal_force = sqrt(normal_force);
    
    
    // the force due to static friction, modeled as horizontal damper, again
    // in object centered coordinates
    tangent_force = 0;
    viscvel = 1.e-10;
    for (i=1; i<=N_CART; ++i) {
      temp[i] = -optr->contact_parms[3] * cptr->tangent[i] - 
	optr->contact_parms[4]*cptr->tanvel[i];
      tangent_force += sqr(temp[i]);
      viscvel += sqr(cptr->viscvel[i]);
    }
    tangent_force = sqrt(tangent_force);
    viscvel = sqrt(viscvel);

    /* If static friction too large -> spring breaks -> dynamic friction in
       the direction of the viscvel vector; we also reset the x_start
       vector such that static friction would be triggered appropriately,
       i.e., when the viscvel becomes zero */
    if (tangent_force > optr->contact_parms[5] * normal_force || cptr->friction_flag) {
      cptr->friction_flag = TRUE;
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += -optr->contact_parms[6] * normal_force * cptr->viscvel[i]/viscvel;
	if (viscvel < 0.01) {
	  cptr->friction_flag = FALSE;
	  cptr->x_start[i] = cptr->x[i];
	}
      } 
    } else {
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += temp[i];
      }
    }
    break;

  case DAMPED_SPRING_VISCOUS_FRICTION:

    /* note: the contact_parms arrays has the following elements:
       contact_parms[1] = normal spring coefficient
       contact_parms[2] = normal damping coefficient
       contact_parms[3] = viscous friction coefficient
    */

    /* normal contact force */
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] = optr->contact_parms[1] * cptr->normal[i] +
	optr->contact_parms[2] * cptr->normvel[i];
      // make sure the damping part does not attract a contact force with wrong sign
      if (macro_sign(cptr->f[i])*macro_sign(cptr->normal[i])<0)
	cptr->f[i] = 0.0;
    }
    
    /* the force due to viscous friction */
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] += -optr->contact_parms[3] * cptr->viscvel[i];
    }
    break;

  case DAMPED_SPRING_LIMITED_REBOUND:
    
    /* note: the contact_parms arrays has the following elements:
       contact_parms[1] = normal spring coefficient
       contact_parms[2] = normal damping coefficient
       contact_parms[3] = static friction spring coefficient
       contact_parms[4] = static friction damping spring coefficient
       contact_parms[5] = static friction coefficient (friction cone)
       contact_parms[6] = dynamic friction coefficient (proportional to normal force)
       contact_parms[7] = normal max rebound velocity
       contact_parms[8] = static friction max rebound velocity
    */
    
    // the normal contact force in object centered coordinates
    normal_velocity = 0.0;
    aux = 0;
    for (i=1; i<=N_CART; ++i) {
      temp1[i] = optr->contact_parms[1] * cptr->normal[i];
      temp2[i] = optr->contact_parms[2] * cptr->normvel[i];
      normal_velocity  += sqr(cptr->normvel[i]);
      aux              += cptr->normvel[i]*cptr->normal[i];
    }
    normal_velocity = -sqrt(normal_velocity)*macro_sign(aux);
    aux = 1.0-normal_velocity/optr->contact_parms[7];
    if (aux < 0)
      aux = 0.0;

    normal_force = 0;
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] = temp1[i]*aux + temp2[i]*sqrt(aux);
      // make sure the damping part does not attract a contact force with wrong sign
      if (macro_sign(cptr->f[i])*macro_sign(cptr->normal[i])<0) 
	cptr->f[i] = 0.0;
      normal_force     += sqr(cptr->f[i]);
    }
    normal_force = sqrt(normal_force);

    
    // the force due to static friction, modeled as horizontal damper, again
    // in object centered coordinates
    tangent_force = 0;
    tangent_velocity = 0;
    viscvel = 1.e-10;
    aux = 0;
    for (i=1; i<=N_CART; ++i) {
      temp1[i] = -optr->contact_parms[3] * cptr->tangent[i];
      temp2[i] = -optr->contact_parms[4]*cptr->tanvel[i];
      tangent_velocity += sqr(cptr->tanvel[i]);
      viscvel          += sqr(cptr->viscvel[i]);
      aux              += (cptr->x[i]-cptr->x_start[i])*cptr->tanvel[i];
    }
    tangent_velocity = -sqrt(tangent_velocity)*macro_sign(aux);
    viscvel          = sqrt(viscvel);

    aux = 1.0-tangent_velocity/optr->contact_parms[8];
    if (aux < 0)
      aux = 0.0;

    tangent_force = 0.0;
    for (i=1; i<=N_CART; ++i) {
      temp[i] = temp1[i]*aux + temp2[i]*sqrt(aux);
      tangent_force    += sqr(temp[i]);
    }
    tangent_force    = sqrt(tangent_force);

    /* If static friction too large -> spring breaks -> dynamic friction in
       the direction of the viscvel vector; we also reset the x_start
       vector such that static friction would be triggered appropriately,
       i.e., when the viscvel becomes zero */
    if (tangent_force > optr->contact_parms[5] * normal_force || cptr->friction_flag) {
      cptr->friction_flag = TRUE;
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += -optr->contact_parms[6] * normal_force * cptr->viscvel[i]/viscvel;
	if (viscvel < 0.01) {
	  cptr->friction_flag = FALSE;
	  cptr->x_start[i] = cptr->x[i];
	}
      } 
    } else {
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += temp[i];
      }
    }
    break;

  default:
    break;

  }

  /* convert the object centered forces into global coordinates */
  if (optr->rot[_G_] != 0.0) {
    aux    =  cptr->f[_X_]*cos(optr->rot[_G_])-cptr->f[_Y_]*sin(optr->rot[_G_]);
    cptr->f[_Y_] =  cptr->f[_X_]*sin(optr->rot[_G_])+cptr->f[_Y_]*cos(optr->rot[_G_]);
    cptr->f[_X_] = aux;
  }

  if (optr->rot[_B_] != 0.0) {
    aux    =  cptr->f[_X_]*cos(optr->rot[_B_])+cptr->f[_Z_]*sin(optr->rot[_B_]);
    cptr->f[_Z_] = -cptr->f[_X_]*sin(optr->rot[_B_])+cptr->f[_Z_]*cos(optr->rot[_B_]);
    cptr->f[_X_] = aux;
  }
  
  if (optr->rot[_A_] != 0.0) {
    aux    =  cptr->f[_Y_]*cos(optr->rot[_A_])-cptr->f[_Z_]*sin(optr->rot[_A_]);
    cptr->f[_Z_] =  cptr->f[_Y_]*sin(optr->rot[_A_])+cptr->f[_Z_]*cos(optr->rot[_A_]);
    cptr->f[_Y_] = aux;
  }

  /* add forces to appropriate DOF and object */
  for (i=1; i<=N_CART; ++i) {
    ucontact[cptr->base_dof].f[i] += cptr->f[i];
    optr->f[i] += cptr->f[i];
    moment_arm[i] = link_pos_sim[ind][i]-link_pos_sim[cptr->off_link][i];
    moment_arm_object[i] = link_pos_sim[ind][i]-optr->trans[i];
  }

  /* get the torque at the DOF from the cross product */
  ucontact[cptr->base_dof].t[_A_] += moment_arm[_Y_]*cptr->f[_Z_] - 
    moment_arm[_Z_]*cptr->f[_Y_];
  ucontact[cptr->base_dof].t[_B_] += moment_arm[_Z_]*cptr->f[_X_] - 
    moment_arm[_X_]*cptr->f[_Z_];
  ucontact[cptr->base_dof].t[_G_] += moment_arm[_X_]*cptr->f[_Y_] - 
    moment_arm[_Y_]*cptr->f[_X_];

  /* get the torque at the object center from the cross product */
  optr->t[_A_] += moment_arm_object[_Y_]*cptr->f[_Z_] - 
    moment_arm_object[_Z_]*cptr->f[_Y_];
  optr->t[_B_] += moment_arm_object[_Z_]*cptr->f[_X_] - 
    moment_arm_object[_X_]*cptr->f[_Z_];
  optr->t[_G_] += moment_arm_object[_X_]*cptr->f[_Y_] - 
    moment_arm_object[_Y_]*cptr->f[_X_];

}

/*!*****************************************************************************
 *******************************************************************************
\note  contactVelocity
\date  March 2006
   
\remarks 

        computes the velocity of a contact point in object coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     cID    : ID of contact point
 \param[in]     optr   : object pointer
 \param[out]    v      : velocity vector

 ******************************************************************************/
static void 
contactVelocity(int cID, ObjectPtr optr, double *v)
{
  double aux;

  // get the velocity in world coordinates
  computeLinkVelocity(cID, link_pos_sim, joint_origin_pos_sim, 
		      joint_axis_pos_sim, joint_sim_state, v);

  // convert the velocity to object coordinates
  if (optr->rot[1] != 0.0) {
    aux  =  v[2]*cos(optr->rot[1])+v[3]*sin(optr->rot[1]);
    v[3] = -v[2]*sin(optr->rot[1])+v[3]*cos(optr->rot[1]);
    v[2] = aux;
  }
  
  if (optr->rot[2] != 0.0) {
    aux  =  v[1]*cos(optr->rot[2])-v[3]*sin(optr->rot[2]);
    v[3] =  v[1]*sin(optr->rot[2])+v[3]*cos(optr->rot[2]);
    v[1] = aux;
  }
  
  if (optr->rot[3] != 0.0) {
    aux  =  v[1]*cos(optr->rot[3])+v[2]*sin(optr->rot[3]);
    v[2] = -v[1]*sin(optr->rot[3])+v[2]*cos(optr->rot[3]);
    v[1] = aux;
  }

}

