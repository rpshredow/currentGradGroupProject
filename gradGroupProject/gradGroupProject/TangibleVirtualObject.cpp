#include <stdlib.h>
#include <math.h>
#include <assert.h>

#if defined(WIN32)
#include <windows.h>
#endif


#include <HL/hl.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>


#include "objloader.h"

using namespace std;

/* Haptic device and rendering context handles. */
static HHD ghHD = HD_INVALID_HANDLE;
static HHLRC ghHLRC = 0;

/* Shape id for shape we will render haptically. */
GLuint displayList;

#define CURSOR_SIZE_PIXELS 20
static double gCursorScale;
static GLuint gCursorDisplayList = 0;
static GLuint gPencilDisplayList = 0;

/* Struct representing one of the shapes in the scene that can be felt, touched and drawn. */
struct HapticObject
{
    HLuint shapeId;
    GLuint displayList;
    hduMatrix transform;
	float hap_stiffness;
    float hap_damping;
    float hap_static_friction;
    float hap_dynamic_friction;
};
std::vector<HapticObject> hapticObjects;
set<int>nearestNeighbour;




hduVector3Dd proxyPosition;
hduVector3Dd devicePosition;
hduVector3Dd constrainedProxy;
hduVector3Dd minPoint;
hduVector3Dd maxPoint;
HLdouble proxyxform[16];
long int gCurrentTouchObj = -1;
long int gCurrentDragObj = -1;
//Display list for model
GLuint objList;

vector<glm::vec3> vertices;
vector<int> indices;
vector<glm::vec3> normals;
vector<double> surfaceFriction;
vector<Triangle> triContainer;
vector<OBJLoader> loaderVec;

float stiffnessCoefficient = 1.0;

HDboolean bRenderForce = HD_FALSE;
HLboolean isAnchoredEditing = false;
HLboolean toggleCursor = false;
HLboolean isProxyConstrained = false;
static HDdouble gSpringStiffness = 0.1;
static HDdouble gMaxStiffness = 1.0;
OBJLoader loaderOne;
OBJLoader loaderTwo;
OBJLoader pencilLoader;
hduVector3Dd newProxyPosition;

hduMatrix initProxyTransform;
hduMatrix initObjTransform;
int nearestID;

int loaderIndex; 
double currentFriction;
HDSchedulerHandle gCallbackHandle = 0;
hduVector3Dd anchor;
hduVector3Dd trueDevicePosition;
hduVector3Dd anchorProxyPosition;
hduVector3Dd anchorInitDevicePosition;
int anchorTouchPoint;

hduVector3Dd transformedProxyPosition;


/* Function prototypes. */
void glutDisplay(void);
void glutReshape(int width, int height);
void glutIdle(void);   
void glutMenu(int); 
void keyboard(unsigned char key, int x, int y);
void exitHandler(void);

void initGL();
void initOBJModel();
void initPROXYModel();
void initHL();
void initScene();
void drawSceneHaptics();
void drawSceneGraphics();
void drawCursor();
void drawPoint();
void drawBox();
void updateWorkspace();
void createHapticObject();



void HLCALLBACK buttonDownClientThreadCallback (HLenum event, HLuint object, HLenum thread, HLcache*cache, void *userdata);
void HLCALLBACK buttonUpClientThreadCallback (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata);
void HLCALLBACK hlTouchCB (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata);
void HLCALLBACK hlUnTouchCB (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata);
void HLCALLBACK hlMotionCB (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata);
HDCallbackCode HDCALLBACK AnchoredSpringForceCallback(void *pUserData);
void updateObjTransform();
void updateDragObjectTransform();
int findNearestVertex(vec3 proxyPosition);

/*******************************************************************************
 Initializes GLUT for displaying a simple haptic scene.
*******************************************************************************/
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(1000, 1000);
    glutCreateWindow("Tangible Virtual Object");

    // Set glut callback functions.
    glutDisplayFunc(glutDisplay);
    glutReshapeFunc(glutReshape);
    glutIdleFunc(glutIdle);
    glutKeyboardFunc(keyboard);
    glutCreateMenu(glutMenu);
    glutAddMenuEntry("Quit", 0);
    glutAttachMenu(GLUT_RIGHT_BUTTON);    
    
    // Provide a cleanup routine for handling application exit.
    atexit(exitHandler);

    initScene();

    glutMainLoop();

    return 0;
}

/*******************************************************************************
 GLUT callback for redrawing the view.
*******************************************************************************/
void glutDisplay()
{   
    drawSceneHaptics();
    drawSceneGraphics();
    glutSwapBuffers();
}

/*******************************************************************************
 GLUT callback for reshaping the window.  This is the main place where the 
 viewing and workspace transforms get initialized.
*******************************************************************************/
void glutReshape(int width, int height)
{
    static const double kPI = 3.1415926535897932384626433832795;
    static const double kFovY = 40;

    double nearDist, farDist, aspect;

    glViewport(0, 0, width, height);

    // Compute the viewing parameters based on a fixed fov and viewing
    // a canonical box centered at the origin.

    nearDist = 1.0 / tan((kFovY / 2.0) * kPI / 180.0);
    farDist = nearDist + 10.0;
    aspect = (double) width / height;
   
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(kFovY, aspect, nearDist, farDist);

    // Place the camera down the Z axis looking at the origin.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();            
    gluLookAt(0, 0, nearDist + 3.0,
              0, 0, 0,
              0, 1, 0);
    
    updateWorkspace();
}

/*******************************************************************************
 GLUT callback for idle state.  Use this as an opportunity to request a redraw.
 Checks for HLAPI errors that have occurred since the last idle check.
*******************************************************************************/
void glutIdle()
{
    HLerror error;

    while (HL_ERROR(error = hlGetError()))
    {
        fprintf(stderr, "HL Error: %s\n", error.errorCode);
        
        if (error.errorCode == HL_DEVICE_ERROR)
        {
            hduPrintError(stderr, &error.errorInfo,
                "Error during haptic rendering\n");
        }
    }
    
    glutPostRedisplay();
}

/******************************************************************************
 Popup menu handler.
******************************************************************************/
void glutMenu(int ID)
{
    switch(ID) {
        case 0:
            exit(0);
            break;
    }
}
/******************************************************************************/
void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	
	case 'a':
	case 'A':
		isAnchoredEditing = !isAnchoredEditing;
		if(isAnchoredEditing && (gCurrentTouchObj != -1 && isProxyConstrained)){
			anchor = trueDevicePosition;//device in physical space
			anchorProxyPosition = proxyPosition;//proxy
			anchorTouchPoint = nearestID;//the point being touched
			anchorInitDevicePosition = devicePosition; // device in virtual space
			bRenderForce = HD_TRUE;

			for(set<int>::iterator cur_b= loaderVec[loaderIndex].net[anchorTouchPoint].begin(); cur_b!= loaderVec[loaderIndex].net[anchorTouchPoint].end(); cur_b++)
						nearestNeighbour.insert(*cur_b);
					}

		
		else{
			nearestNeighbour.clear();
			bRenderForce = HD_FALSE;
			}
		break;

	case 't':
	case 'T':
		
		toggleCursor = !toggleCursor;
		
		break;
	case 'e':
	case 'E':
		isProxyConstrained = !isProxyConstrained;
		if(isProxyConstrained){
			constrainedProxy = proxyPosition;
			//constrain the proxy inside the box but we aren't drawing the box here. 
			minPoint[0] = constrainedProxy[0]-.25;
			minPoint[1] = constrainedProxy[1]-.25;
			minPoint[2] = constrainedProxy[2]-.25;

			maxPoint[0] = constrainedProxy[0]+.25;
			maxPoint[1] = constrainedProxy[1]+.25;
			maxPoint[2] = constrainedProxy[2]+.25;
		}
		updateWorkspace();
		
	}

}

/*******************************************************************************
 Initializes the scene.  Handles initializing both OpenGL and HL.
*******************************************************************************/
void initScene()
{
	
	initOBJModel();
	initPROXYModel();	
    initGL();
    initHL();
	createHapticObject();
}

/*******************************************************************************/

void initPROXYModel(){

	bool loadpencilFile = pencilLoader.load("pencil.obj");

	printf("Number of vertices is: %d\n", (pencilLoader.getVertices()).size());

}

void initOBJModel(){
	


	bool loadfileOne = loaderOne.load("WavySurface.obj");
	vertices = loaderOne.getVertices();
	normals = loaderOne.getNormals();
	surfaceFriction = loaderOne.getFriction();
	indices = loaderOne.getVertexIndices();
	triContainer = loaderOne.getTriangles();

	loaderVec.push_back(loaderOne);

	vertices.clear();
	normals.clear();
	surfaceFriction.clear();
	indices.clear();
	triContainer.clear();

	bool loadfilTwo = loaderTwo.load("swq.obj");
	vertices = loaderTwo.getVertices();
	normals = loaderTwo.getNormals();
	surfaceFriction = loaderTwo.getFriction();
	indices = loaderTwo.getVertexIndices();
	triContainer = loaderTwo.getTriangles();

	loaderVec.push_back(loaderTwo);
}

/*******************************************************************************
 Sets up general OpenGL rendering properties: lights, depth buffering, etc.
*******************************************************************************/
void initGL()
{
    static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    static const GLfloat light0_diffuse[] = {0.9f, 0.9f, 0.9f, 0.9f};   
    static const GLfloat light0_direction[] = {0.0f, -0.4f, 1.0f, 0.0f};    
    
    // Enable depth buffering for hidden surface removal.
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    
    // Cull back faces.
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    
    // Setup other misc features.
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    
    // Setup lighting model.
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
    glEnable(GL_LIGHT0);   
}

/*******************************************************************************
 Initialize the HDAPI.  This involves initing a device configuration, enabling
 forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void initHL()
{
    HDErrorInfo error;

	/* Start the haptic rendering loop. */

    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "Press any key to exit");
        getchar();
        exit(-1);
    }
	
	gCallbackHandle = hdScheduleAsynchronous(AnchoredSpringForceCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    hdEnable(HD_FORCE_OUTPUT);

    ghHLRC = hlCreateContext(ghHD);
    hlMakeCurrent(ghHLRC);
    hlEnable(HL_HAPTIC_CAMERA_VIEW);

	
}

/*******************************************************************************
 This handler is called when the application is exiting.  Deallocates any state 
 and cleans up.
*******************************************************************************/
void exitHandler()
{
    // Deallocate the sphere shape id we reserved in initHL.
    
	for(int i=0; hapticObjects.size(); i++ )
		hlDeleteShapes(hapticObjects[i].shapeId, 1);

    // Free up the haptic rendering context.
    hlMakeCurrent(NULL);
    if (ghHLRC != NULL)
    {
        hlDeleteContext(ghHLRC);
    }

    hdUnschedule(gCallbackHandle);
    // Free up the haptic device.
    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
    }
}

/*******************************************************************************
 Use the current OpenGL viewing transforms to initialize a transform for the
 haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void updateWorkspace()
{
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    hlMatrixMode(HL_TOUCHWORKSPACE);
    hlLoadIdentity();
    
    // Fit haptic workspace to view volume.
	if(!isProxyConstrained)
		hluFitWorkspace(projection);
	else
		hluFitWorkspaceBox(modelview, minPoint,maxPoint);
    // Compute cursor scale.
    gCursorScale = hluScreenToModelScale(modelview, projection, viewport);
    gCursorScale *= CURSOR_SIZE_PIXELS;
}

/*******************************************************************************/
void createHapticObject(){
	//I don't this the problem is here.

	HapticObject hapticObject;
    hapticObject.hap_stiffness = 0.8;
    hapticObject.hap_damping = 0.0;
    hapticObject.hap_static_friction = 0.5;
    hapticObject.hap_dynamic_friction = 0.0;
    
	hapticObject.shapeId = hlGenShapes(1);
    hapticObject.transform = hduMatrix::createTranslation(0,0,0);
    hapticObject.displayList = glGenLists(1);
	hapticObjects.push_back(hapticObject); //first object
	

	//multiple objects code
	
	
	//printf("%f\n", hapticObjects.size());
	
	/*glNewList(hapticObject.displayList, GL_COMPILE);
        glutSolidCube(40);
    glEndList();
	*/
	hapticObject.hap_stiffness = 0.8;
    hapticObject.hap_damping = 0.0;
    hapticObject.hap_static_friction = 0.5;
    hapticObject.hap_dynamic_friction = 0.0;

	hapticObject.shapeId = hlGenShapes(1);
    hapticObject.transform = hduMatrix::createTranslation(1,1,-2);
    hapticObject.displayList = glGenLists(1);
	
	
	hapticObjects.push_back(hapticObject);

	//printf("%f\n", hapticObjects.size());




	// loop for multiple objects, works with original code
   	for ( int i = 0; i < hapticObjects.size(); i++){
		//cout<<i;
	      printf("%d\n", hapticObjects[i].shapeId);
	      hlAddEventCallback(HL_EVENT_1BUTTONDOWN, hapticObjects[i].shapeId, HL_CLIENT_THREAD, buttonDownClientThreadCallback,0);
	      hlAddEventCallback(HL_EVENT_1BUTTONUP, HL_OBJECT_ANY, HL_CLIENT_THREAD, buttonUpClientThreadCallback, 0); 
	      hlAddEventCallback(HL_EVENT_TOUCH, hapticObjects[i].shapeId, HL_COLLISION_THREAD, hlTouchCB, 0); 
	      hlAddEventCallback(HL_EVENT_UNTOUCH, HL_OBJECT_ANY, HL_COLLISION_THREAD, hlUnTouchCB, 0);
	      hlAddEventCallback(HL_EVENT_MOTION, hapticObjects[i].shapeId, HL_COLLISION_THREAD, hlMotionCB, 0);
	}


}
/*******************************************************************************
 The main routine for displaying the scene.  Gets the latest snapshot of state
 from the haptic thread and uses it to display a 3D cursor.
*******************************************************************************/
void drawSceneGraphics()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);           
	glEnable(GL_COLOR_MATERIAL);
    // Draw 3D cursor at haptic device position.
    drawCursor();
	if(isProxyConstrained) drawBox();
	
	/*
	// orginial code
	glPushMatrix();
    // Draw a cube using OpenGL.
	//glPolygonMode(GL_FRONT, GL_LINE);
	glMultMatrixd(hapticObject.transform);
	//glCallList(displayList);
    loader.drawColorObj();
	if(gCurrentTouchObj==hapticObject.shapeId) drawPoint();
	glPopMatrix();
	*/

	//multiple objects code
	
	for (int i = 0; i < hapticObjects.size(); i++){
	
			glPushMatrix();
			glMultMatrixd(hapticObjects[i].transform);
			
				loaderVec[i].drawColorObj();
			
			glPopMatrix();

	}

	glPushMatrix();
	glMultMatrixd(hapticObjects[loaderIndex].transform);
		drawPoint();
		
		glPopMatrix();
	

	

	

}

/*******************************************************************************
 The main routine for rendering scene haptics.
*******************************************************************************/
void drawSceneHaptics()
{    
	
    // Start haptic frame.  (Must do this before rendering any haptic shapes.)
    hlBeginFrame();
	
	hlCheckEvents();
	

	HLboolean buttDown;
    hlGetBooleanv(HL_BUTTON1_STATE, &buttDown);

	hlTouchModel(HL_CONTACT);
	hlTouchableFace(HL_FRONT);
	
	if(buttDown && gCurrentDragObj !=-1)
		updateDragObjectTransform();
		//updateObjTransform();
	

	
    // Start a new haptic shape.  Use the feedback buffer to capture OpenGL geometry for haptic rendering.
	
	//original code
	/*
	if(gCurrentDragObj == -1){
		glPushMatrix();
        // Use OpenGL commands to create geometry.
		glMultMatrixd( hapticObject.transform);
		hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER,hapticObject.shapeId );
        // End the shape.
		loader.drawColorObj();
		//glCallList(displayList);
        hlEndShape();
		glPopMatrix();
	}
	*/

	// multiple objects code
	
	for (int i = 0; i < hapticObjects.size(); i++){
        // Position and orient the object.

		
        glPushMatrix();

        glMultMatrixd(hapticObjects[i].transform);
		
		
            
        // Draw the object haptically (but not if it is being dragged).
        if (hapticObjects[i].shapeId != gCurrentDragObj ){
			hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, hapticObjects[i].hap_stiffness);
            hlMaterialf(HL_FRONT, HL_DAMPING, hapticObjects[i].hap_damping);
            hlMaterialf(HL_FRONT, HL_STATIC_FRICTION, hapticObjects[i].hap_static_friction);
            hlMaterialf(HL_FRONT, HL_DYNAMIC_FRICTION, hapticObjects[i].hap_dynamic_friction);
			
            hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, hapticObjects[i].shapeId);

			loaderVec[i].drawColorObj();

            //glCallList(hapticObjects[i].displayList);

            hlEndShape();
        }
        glPopMatrix();
    }
	


    // End the haptic frame.

    hlEndFrame();
}


/*******************************************************************************
 Draws a 3D cursor for the haptic device using the current local transform,
 the workspace to world transform and the screen coordinate scale.
*******************************************************************************/
void drawCursor()
{
	static const double kCursorRadius = 0.5;
    static const double kCursorHeight = 1.5;
    static const int kCursorTess = 15;
   
	HLdouble proxyxform[16];

	hlGetDoublev(HL_PROXY_POSITION, proxyPosition);
	hlGetDoublev(HL_DEVICE_POSITION, devicePosition);
	
    GLUquadricObj *qobj = 0;

    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glPushMatrix();

   if (!gCursorDisplayList)
   {
		gCursorDisplayList = glGenLists(1);
        glNewList(gCursorDisplayList, GL_COMPILE);
        qobj = gluNewQuadric();
               
		gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight, kCursorTess, kCursorTess);
		glTranslated(0.0, 0.0, kCursorHeight);
		gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0, kCursorTess, kCursorTess);
    
		gluDeleteQuadric(qobj);
		glEndList();
   }

   if (!gPencilDisplayList)
   {
		
		gPencilDisplayList = glGenLists(1);
        glNewList(gPencilDisplayList, GL_COMPILE);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glTranslated(0.0, 1.0, 0.0);
		pencilLoader.drawColorObj();
		glEndList();
   }
    
    
    // Get the proxy transform in world coordinates.
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
    // Get the proxy transform in world coordinates.
	if(bRenderForce){
		proxyxform[12] = newProxyPosition[0];
		proxyxform[13] = newProxyPosition[1];
		proxyxform[14] = newProxyPosition[2];
	}
    glMultMatrixd(proxyxform);

   
    
	//glEnable(GL_COLOR_MATERIAL);
    
	// Apply the local cursor scale factor.
	
	if(toggleCursor){
		glCallList(gPencilDisplayList);
		//glCallList(gCursorDisplayList);
	}
	else {
		glColor3f(0.0, 0.5, 1.0);
		glScaled(gCursorScale, gCursorScale, gCursorScale);
		//glCallList(gPencilDisplayList);
		glCallList(gCursorDisplayList);
	}
   
    glPopMatrix(); 
    glPopAttrib();
}
void HLCALLBACK buttonDownClientThreadCallback (HLenum event, HLuint object, HLenum thread, HLcache*cache, void *userdata){
	//if the cursor is touching the cube allow the user to grab
	//if(gCurrentTouchObj != -1){
	for (int i = 0; i < hapticObjects.size(); i++){
		if (hapticObjects[i].shapeId == object)
			loaderIndex = i;

	}

		gCurrentDragObj = object;
		printf("gCurrentDragObj is //: %i\n", gCurrentDragObj);//We don't know why this is happening. 
		
	//}	
	//if(gCurrentTouchObj == hapticObject.shapeId)
		//gCurrentDragObj = object;
	hlGetDoublev(HL_PROXY_TRANSFORM, initProxyTransform);
	//initObjTransform = hapticObject.transform;

	initObjTransform = hapticObjects[loaderIndex].transform;
	printf("Loader Index: %i\n", loaderIndex);

	
}
void HLCALLBACK buttonUpClientThreadCallback (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata){
	if(gCurrentDragObj != -1)
		gCurrentDragObj = -1;
}
void HLCALLBACK hlTouchCB (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata){
	gCurrentTouchObj = object;
	cout<<"Current touch obj= "<< gCurrentTouchObj << endl;
	
	

	vec3 tProxyPos;

	hduMatrix mat;
	for (int i = 0; i < hapticObjects.size(); i++){
		
		if (hapticObjects[i].shapeId == object){
			loaderIndex = i;
			mat = (hapticObjects[i].transform).getInverse();
			cout<< "We made into the if statement for touch obj"<< endl;
			mat.multVecMatrix(proxyPosition, transformedProxyPosition);
			tProxyPos[0] = transformedProxyPosition[0];
			tProxyPos[1] = transformedProxyPosition[1];
			tProxyPos[2] = transformedProxyPosition[2];
		}
	}

	printf("Touch Transformed Proxy Position x: %d, y: %d, z: %d\n", tProxyPos[0], tProxyPos[1], tProxyPos[2]);
	printf("Touch Proxy Position x: %d, y: %d, z: %d\n", proxyPosition[0], proxyPosition[1], proxyPosition[2]);

	nearestID = findNearestVertex(tProxyPos);
	
	currentFriction =loaderVec[loaderIndex].getFriction()[nearestID];
}
void HLCALLBACK hlUnTouchCB (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata){
	if(gCurrentTouchObj != -1)
		gCurrentTouchObj = -1;
}
void HLCALLBACK hlMotionCB (HLenum event, HLuint object, HLenum thread, HLcache*cache, void*userdata){
	gCurrentTouchObj = object;
	//printf ("%i", touch);
	//hduVector3Dd transformedProxyPosition;
	hduMatrix mat;
	vec3 tProxyPos;
	for (int i = 0; i < hapticObjects.size(); i++){
		if (hapticObjects[i].shapeId == object){
			loaderIndex= i;
			mat = (hapticObjects[i].transform).getInverse();
			mat.multVecMatrix(proxyPosition, transformedProxyPosition);
			tProxyPos[0] = transformedProxyPosition[0];
			tProxyPos[1] = transformedProxyPosition[1];
			tProxyPos[2] = transformedProxyPosition[2];
			
		}
	}
	
	//hduMatrix mat = (hapticObject.transform).getInverse();

	printf("Motion Transformed Proxy Position x: %d, y: %d, z: %d\n", tProxyPos[0], tProxyPos[1], tProxyPos[2]);
	
	nearestID = findNearestVertex(tProxyPos);
	currentFriction = loaderVec[loaderIndex].getFriction()[nearestID];
	
	//printf("%d ", nearestID);
	
	//printf("%f\n", currentFriction);
}
/*
void updateObjTransform(){
	hduMatrix currentProxyTransform;
	hduVector3Dd currentProxyTranslation;
	hduVector3Dd initProxyTranslation;

	hduMatrix initRotation;
	hduMatrix currentRotation;

	hlGetDoublev(HL_PROXY_TRANSFORM,currentProxyTransform);
	//getting the current translation vector 
	currentProxyTranslation[0] = currentProxyTransform[3][0];
	currentProxyTranslation[1] = currentProxyTransform[3][1];
	currentProxyTranslation[2] = currentProxyTransform[3][2];
	//getting the inital translation vector 
	initProxyTranslation[0] = initProxyTransform[3][0];
	initProxyTranslation[1] = initProxyTransform[3][1];
	initProxyTranslation[2] = initProxyTransform[3][2];
	//getting inital rotation matrix
	initRotation = initProxyTransform; 
	initRotation[3][0]=0.0;
	initRotation[3][1]=0.0;
	initRotation[3][2]=0.0;
	//getting current rotation matrix
	currentRotation = currentProxyTransform;
	currentRotation[3][0]=0.0;
	currentRotation[3][1]=0.0;
	currentRotation[3][2]=0.0;

	hduVector3Dd deltaTranslation = currentProxyTranslation - initProxyTranslation;

	hduMatrix deltaRotationMatrix = initRotation.getInverse()*currentRotation;

	hduMatrix deltaTransformation = hduMatrix::createTranslation(deltaTranslation);

	hduMatrix toCenter = hduMatrix::createTranslation(-initProxyTranslation);

	hduMatrix fromCenter = hduMatrix::createTranslation(initProxyTranslation);

	hduMatrix overallDeltaRotation = toCenter*deltaRotationMatrix*fromCenter;

	deltaTransformation = overallDeltaRotation*deltaTransformation;

	hapticObject.transform = initObjTransform*deltaTransformation;

	for (int i = 0; i < hapticObjects.size(); i++){
		if (hapticObjects[i].shapeId == gCurrentDragObj){
	        hapticObjects[i].transform = currentProxyTransform*deltaTransformation;
		    break;
		}
	}

}
*/

void updateDragObjectTransform(){
	//assert(gCurrentDragObj >= 0);
	
	hduMatrix proxyxform;
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);

	// Translation part

	hduVector3Dd proxyPos(proxyxform[3][0], proxyxform[3][1], proxyxform[3][2] );
	hduVector3Dd gstartDragProxyPos(initProxyTransform[3][0], initProxyTransform[3][1], initProxyTransform[3][2]);

	hduVector3Dd dragDeltaTransl = proxyPos - gstartDragProxyPos;

	// Rotation part

	// Same for rotation.
    
	hduMatrix deltaRotMat;
	
	hduMatrix gstartDragProxyRotation;
    gstartDragProxyRotation = initProxyTransform;
	gstartDragProxyRotation[3][0] = 0.0;
	gstartDragProxyRotation[3][1] = 0.0;
	gstartDragProxyRotation[3][2] = 0.0;

	hduMatrix proxyRot = proxyxform;
	proxyRot[3][0] = 0.0;
	proxyRot[3][1] = 0.0;
	proxyRot[3][2] = 0.0;

	deltaRotMat = gstartDragProxyRotation.getInverse()  * proxyRot;

	// If the object is not centered

	hduMatrix toProxy = hduMatrix::createTranslation(-gstartDragProxyPos);
	hduMatrix fromProxy = hduMatrix::createTranslation(gstartDragProxyPos);
    deltaRotMat = toProxy * deltaRotMat * fromProxy;

	// Compose rotation and translation deltas.
    
	hduMatrix deltaMat = deltaRotMat * hduMatrix::createTranslation(dragDeltaTransl);

	//hduMatrix deltaMat = hduMatrix::createTranslation(dragDeltaTransl);
	
	// Apply these deltas to the drag object transform.
    
	for (int i = 0; i < hapticObjects.size(); i++){
		
		if ( hapticObjects[i].shapeId == gCurrentDragObj){
	        hapticObjects[i].transform = initObjTransform*deltaMat;
		    break;
		}
	}

}

int findNearestVertex(vec3 proxyPosition){
	int nearestPoint = -1;//we're going to return this later when we find the closest point
	double distance;
	double shortestDistance = 1000000;
	int i;
	vector<vec3> vContainer = loaderVec[loaderIndex].getVertices();

	for(i=0; i< vContainer.size(); i++)//outer loop
	{ 
		
			vec3 myNeighbouringPoint = vContainer[i];
			
			//distance formula YAY!!
			distance=sqrt(pow(proxyPosition.x-myNeighbouringPoint.x,2)+pow(proxyPosition.y-myNeighbouringPoint.y,2)+pow(proxyPosition.z-myNeighbouringPoint.z,2));
			if(distance < shortestDistance){
				shortestDistance = distance;
				nearestPoint=i;
		}
	}
	return nearestPoint;
}

void drawPoint(){
	glPointSize(10.0f);
	glBegin(GL_POINTS);
	{
		glColor3f(1.0,1.0,0.0);
		vec3 vert = loaderVec[loaderIndex].getVertices()[nearestID];
		//vec3 vert = loaderVec[loaderIndex].getVertices()[nearestID];
		glVertex3f(vert[0],vert[1],vert[2]);
	}

	glEnd();
}

void drawBox(){
	//vec3 minPoint(min_x, min_y, min_z);
	//vec3 maxPoint(max_x, max_y, max_z);
	

	vec3 Point0(minPoint[0], minPoint[1], minPoint[2]);
	vec3 Point1(minPoint[0], minPoint[1], maxPoint[2]);
	vec3 Point2(maxPoint[0], minPoint[1], maxPoint[2]);
	vec3 Point3(maxPoint[0], minPoint[1], minPoint[2]);
	vec3 Point4(maxPoint[0], maxPoint[1], minPoint[2]);
	vec3 Point5(maxPoint[0], maxPoint[1], maxPoint[2]);
	vec3 Point6(minPoint[0], maxPoint[1], maxPoint[2]);
	vec3 Point7(minPoint[0], maxPoint[1], minPoint[2]);

	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);

	glLineWidth(10.0f);
	glEnable(GL_COLOR_MATERIAL);
	glBegin(GL_LINES);
	glColor3f(1.0,0.0,0.0);
	
	glVertex3f(Point0[0],Point0[1],Point0[2]);
	glVertex3f(Point1[0],Point1[1],Point1[2]);

	glVertex3f(Point1[0],Point1[1],Point1[2]);
	glVertex3f(Point2[0],Point2[1],Point2[2]);

	glVertex3f(Point2[0],Point2[1],Point2[2]);
	glVertex3f(Point3[0],Point3[1],Point3[2]);

	glVertex3f(Point3[0],Point3[1],Point3[2]);
	glVertex3f(Point0[0],Point0[1],Point0[2]);
	//drawing base
	glVertex3f(Point0[0],Point0[1],Point0[2]);
	glVertex3f(Point7[0],Point7[1],Point7[2]);

	glVertex3f(Point1[0],Point1[1],Point1[2]);
	glVertex3f(Point6[0],Point6[1],Point6[2]);

	glVertex3f(Point2[0],Point2[1],Point2[2]);
	glVertex3f(Point5[0],Point5[1],Point5[2]);

	glVertex3f(Point3[0],Point3[1],Point3[2]);
	glVertex3f(Point4[0],Point4[1],Point4[2]);
	//drawing verticles
	glVertex3f(Point7[0],Point7[1],Point7[2]);
	glVertex3f(Point6[0],Point6[1],Point6[2]);

	glVertex3f(Point6[0],Point6[1],Point6[2]);
	glVertex3f(Point5[0],Point5[1],Point5[2]);

	glVertex3f(Point5[0],Point5[1],Point5[2]);
	glVertex3f(Point4[0],Point4[1],Point4[2]);

	glVertex3f(Point4[0],Point4[1],Point4[2]);
	glVertex3f(Point7[0],Point7[1],Point7[2]);
	//drawing top
	glEnd();



	glPopAttrib();
}

HDCallbackCode HDCALLBACK AnchoredSpringForceCallback(void *pUserData){
	hduVector3Dd force(0, 0, 0);
	hduVector3Dd newModelPosition;
	HDErrorInfo error;
	hdBeginFrame(hdGetCurrentDevice());
	hdGetDoublev(HD_CURRENT_POSITION, trueDevicePosition);
	vec3 newVEC3ProxyPosition;
	hduVector3Dd devDifference;
	hduMatrix mat;
	if (bRenderForce){
		// Compute force to be sent to the device here!!
		devDifference = devicePosition - anchorInitDevicePosition;
		newProxyPosition = anchorProxyPosition + devDifference;
		
		
			
				mat= (hapticObjects[loaderIndex].transform).getInverse();
				mat.multVecMatrix(newProxyPosition, newModelPosition);

				newVEC3ProxyPosition[0] = newModelPosition[0];
				newVEC3ProxyPosition[1] = newModelPosition[1];
				newVEC3ProxyPosition[2] = newModelPosition[2];
				
		
				loaderVec[loaderIndex].deformSurface(anchorTouchPoint, newVEC3ProxyPosition, nearestNeighbour);
			
		force = (anchor-trueDevicePosition)*gSpringStiffness;//compute the force

		hdSetDoublev(HD_CURRENT_FORCE, force);	
	}
	hdEndFrame(hdGetCurrentDevice());

	if (HD_DEVICE_ERROR(error = hdGetError())) {
		if (hduIsForceError(&error)) {
			bRenderForce= HD_FALSE;
	}
	else if (hduIsSchedulerError(&error)) {
		return HD_CALLBACK_DONE;
		}
	}
	return HD_CALLBACK_CONTINUE;
}

/******************************************************************************/