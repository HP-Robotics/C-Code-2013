#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "NetworkTables/NetworkTable.h"
 
/**
 * Sample program to use NIVision to find rectangles in the scene that are illuminated
 * by a LED ring light (similar to the model from FIRSTChoice). The camera sensitivity
 * is set very low so as to only show light sources and remove any distracting parts
 * of the image.
 * 
 * The CriteriaCollection is the set of criteria that is used to filter the set of
 * rectangles that are detected. In this example we're looking for rectangles with
 * a minimum width of 30 pixels and maximum of 400 pixels.
 * 
 * The algorithm first does a color threshold operation that only takes objects in the
 * scene that have a bright green color component. Then a convex hull operation fills 
 * all the rectangle outlines (even the partially occluded ones). Then a small object filter
 * removes small particles that might be caused by green reflection scattered from other 
 * parts of the scene. Finally all particles are scored on rectangularity, aspect ratio,
 * and hollowness to determine if they match the target.
 *
 * Look in the VisionImages directory inside the project that is created for the sample
 * images as well as the NI Vision Assistant file that contains the vision command
 * chain (open it with the Vision Assistant)
 */

//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640
//#define VIEW_ANGLE 48		//Axis 206 camera
#define VIEW_ANGLE 43.5  //Axis M1011 camera
#define PI 3.141592653

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double xMin[XMINSIZE] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
const double yMax[YMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double yMin[YMINSIZE] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .6, 0};

//Structure to represent the scores for the various tests used for target identification
struct Scores {
	double rectangularity;
	double aspectRatioInner;
	double aspectRatioOuter;
	double xEdge;
	double yEdge;

};

int start_cpp_task(UINT32 obj);

class Vision2823
{
private:
	Scores *scores;
	//NetworkTable *visionScores;
	Threshold threshold;	
	ParticleFilterCriteria2 criteria[1];
	Task *task;
	double delay;
	
	bool running;
	bool updated;
	
public:
	void Start(void);
	void Stop(void);
	bool isHighGoal;
	double highCenterXNormal;
	double highCenterYNormal;
	double highDistance;
	double highDistance2;
	int highX;
	int highY;
	int highWidth;
	int highHeight;
	
	
	bool isMidGoal;
	double midCenterX;
	double midCenterY;
	double midDistance;
	double midDistance2;
	
	int Run(void);
	Vision2823(double in_delay) : threshold(60, 130, 90, 255, 20, 255) //HSV threshold criteria, ranges are in that order ie. Hue is 60-100
	{
		
		criteria[0].parameter = IMAQ_MT_AREA;
		criteria[0].lower = AREA_MINIMUM;
		criteria[0].upper = 65535;
		criteria[0].calibrated = false;
		criteria[0].exclude = false;
		//visionScores = NetworkTable::GetTable("Vision");
		//visionScores->PutBoolean("VisionTracking", true);
		delay = in_delay;
		task = new Task("vision", (FUNCPTR)(&start_cpp_task));
		updated = false;
	};
	
	bool Updated(void)
	{
		return updated;
	}
	
	void Clear(void)
	{
		updated = false;
	}
};

