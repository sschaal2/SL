/*!=============================================================================
  ==============================================================================

  \file    SL_objects.h

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  SL_objects.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_objects_
#define _SL_objects_


#include "SL_objects_defines.h"


/*! structure to create objects in the environment */
typedef struct Object {
  char    name[STRING100];               /*!< object name */
  int     type;                          /*!< object type */
  int     contact_model;                 /*!< which contact model to be used */
  double  trans[N_CART+1];               /*!< translatory offset of object */
  double  rot[N_CART+1];                 /*!< rotational offset of object */
  double  scale[N_CART+1];               /*!< scaling in x,y,z */
  double  rgb[N_CART+1];                 /*!< color information */
  double  *contact_parms;                /*!< contact parameters */
  double  *object_parms;                 /*!< object parameters */
  char   *nptr;                          /*!< pointer to next object */
  double  f[N_CART+1];                   /*!< forces acting on object in world coordinates */
  double  t[N_CART+1];                   /*!< torques acting on object in world coordinates */
  int     display_list_active;           /*!< display list for open GL (only for terrains) */
  int     hide;                          /*!< allows hiding the object in the display */
} Object, *ObjectPtr;


/*! structure to deal with contact forces */
typedef struct Contact {
  int        active;                           /*!< TRUE/FALSE: indicates whether this
						  point should be checked for contacts */
  int        status;                           /*!< contact is true or false */
  int        friction_flag;                    /*!< flag for switching between different friction models */
  ObjectPtr  optr;                             /*!< ptr of object that is contacted */
  int        base_dof;                         /*!< to which DOF does this point
						  connect? */
  int        off_link;                         /*!< which link should be used for
						  moment arm */
  double     x[N_CART+1];                      /*!< point of contact in object 
						  coordintates */
  double     x_start[N_CART+1];                /*!< point of first contact */
  double     normal[N_CART+1];                 /*!< normal displacement vector */
  double     normvel[N_CART+1];                /*!< normal velocity vector */
  double     tangent[N_CART+1];                /*!< tangential displacement vector */
  double     tanvel[N_CART+1];                 /*!< tangential velocity vector */
  double     viscvel[N_CART+1];                /*!< velocity vector for viscous friction */
  double     f[N_CART+1];                      /*!< contact forces in world coordinates */
  double     face_index;                       /*!< _X_, _Y_, or _Z_ to indicate with which face we are in contact */

						  
} Contact, *ContactPtr;

#ifdef __cplusplus
extern "C" {
#endif

  // external variables

  // shared functions
  int initObjects(void);

  ObjectPtr  addSphere(char *name, double *rgb, double *pos, double *rot, 
		       double *scale, int contact,
		       double *parms, int n_faces);
  ObjectPtr  addCube(char *name, double *rgb, double *pos, double *rot, 
		     double *scale, int contact,
		     double *parms);
  ObjectPtr  addCylinder(char *name, double *rgb, double *pos, double *rot, 
			 double *scale, int contact,
			 Vector parms, int n_faces);
  void       checkContacts(void);
  int        changeObjPosByName(char *name, double *pos, double *rot);
  void       changeObjPosByPtr(ObjectPtr ptr, double *pos, double *rot);
  int        deleteObjByName(char *name);
  void       readObjects(char *fname);
  ObjectPtr  getObjPtrByName(char *name);
  int        getObjForcesByName(char *name, double *f, double *t);
  int        getObjForcesByPtr(ObjectPtr ptr, double *f, double *t);
  int        changeHideObjByName(char *name, int hide);


  // external variables
  extern ObjectPtr  objs;
  extern ContactPtr contacts;
  extern SL_uext   *ucontact;

  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_objects_ */
