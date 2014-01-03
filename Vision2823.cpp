#include "WPILib.h"
#include "Vision2823.h"
/**
 * Image processing code to identify 2013 Vision targets
 */

/**
 * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
 * 
 * @param image The image to use for measuring the particle estimated rectangle
 * @param report The Particle Analysis Report for the particle
 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
 * @return The estimated distance to the target in Inches.
 */
double computeDistance (BinaryImage *image, ParticleAnalysisReport *report, bool outer) {
	double rectShort, height;
	int targetHeight;
	
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
	//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
	//on skewed rectangles
	height = min(report->boundingRect.height, rectShort);
	targetHeight = outer ? 29 : 21;
	
	return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
}


double computeDistance2 (BinaryImage *image, ParticleAnalysisReport *report)
{	
	double pixelsToFeet = 5.146 / report->boundingRect.width;
	double pixelDistance = X_IMAGE_RES / (2 * tan(VIEW_ANGLE * PI / 360));
	return pixelDistance * pixelsToFeet;	
}



/**
 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
 * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
 * and particle perimeter= 2x+2y
 * 
 * @param image The image containing the particle to score, needed to perform additional measurements
 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
 * @return The aspect ratio score (0-100)
 */
double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool outer){
	double rectLong, rectShort, idealAspectRatio, aspectRatio;
	idealAspectRatio = outer ? (62/29) : (62/20);	//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
	
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
	
	//Divide width by height to measure aspect ratio
	if(report->boundingRect.width > report->boundingRect.height){
		//particle is wider than it is tall, divide long by short
		aspectRatio = 100*(1-fabs((1-((rectLong/rectShort)/idealAspectRatio))));
	} else {
		//particle is taller than it is wide, divide short by long
		aspectRatio = 100*(1-fabs((1-((rectShort/rectLong)/idealAspectRatio))));
	}
	return (max(0, min(aspectRatio, 100)));		//force to be in range 0-100
}

/**
 * Compares scores to defined limits and returns true if the particle appears to be a target
 * 
 * @param scores The structure containing the scores to compare
 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
 * 
 * @return True if the particle meets all limits, false otherwise
 */
bool scoreCompare(Scores scores, bool outer){
	bool isTarget = true;

	isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
	if(outer){
		isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
	} else {
		isTarget &= scores.aspectRatioInner > ASPECT_RATIO_LIMIT;
	}
	isTarget &= scores.xEdge > X_EDGE_LIMIT;
	isTarget &= scores.yEdge > Y_EDGE_LIMIT;

	return isTarget;
}

/**
 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
 * 
 * @param report The Particle Analysis Report for the particle to score
 * @return The rectangularity score (0-100)
 */
double scoreRectangularity(ParticleAnalysisReport *report){
	if(report->boundingRect.width*report->boundingRect.height !=0){
		return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
	} else {
		return 0;
	}	
}

/**
 * Computes a score based on the match between a template profile and the particle profile in the X direction. This method uses the
 * the column averages and the profile defined at the top of the sample to look for the solid vertical edges with
 * a hollow center.
 * 
 * @param image The image to use, should be the image before the convex hull is performed
 * @param report The Particle Analysis Report for the particle
 * 
 * @return The X Edge Score (0-100)
 */
double scoreXEdge(BinaryImage *image, ParticleAnalysisReport *report){
	double total = 0;
	LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(), IMAQ_COLUMN_AVERAGES, report->boundingRect);
	for(int i=0; i < (averages->columnCount); i++){
		if(xMin[i*(XMINSIZE-1)/averages->columnCount] < averages->columnAverages[i] 
		   && averages->columnAverages[i] < xMax[i*(XMAXSIZE-1)/averages->columnCount]){
			total++;
		}
	}
	total = 100*total/(averages->columnCount);		//convert to score 0-100
	imaqDispose(averages);							//let IMAQ dispose of the averages struct
	return total;
}

/**
 * Computes a score based on the match between a template profile and the particle profile in the Y direction. This method uses the
 * the row averages and the profile defined at the top of the sample to look for the solid horizontal edges with
 * a hollow center
 * 
 * @param image The image to use, should be the image before the convex hull is performed
 * @param report The Particle Analysis Report for the particle
 * 
 * @return The Y Edge score (0-100)
 */
double scoreYEdge(BinaryImage *image, ParticleAnalysisReport *report){
	double total = 0;
	LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(), IMAQ_ROW_AVERAGES, report->boundingRect);
	for(int i=0; i < (averages->rowCount); i++){
		if(yMin[i*(YMINSIZE-1)/averages->rowCount] < averages->rowAverages[i] 
		   && averages->rowAverages[i] < yMax[i*(YMAXSIZE-1)/averages->rowCount]){
			total++;
		}
	}
	total = 100*total/(averages->rowCount);		//convert to score 0-100
	imaqDispose(averages);						//let IMAQ dispose of the averages struct
	return total;
}		

 int Vision2823::Run()
{
	/**
	 * Do the image capture with the camera and apply the algorithm described above. This
	 * sample will either get images from the camera or from an image file stored in the top
	 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
	 */
	 
	AxisCamera &camera = AxisCamera::GetInstance("10.28.23.11");	//To use the Axis camera uncomment this line
    
	while (running)
	{
		ColorImage *image = camera.GetImage();
		//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
		//visionScores->PutNumber("get image result", camera.GetImage(image));				//To get the images from the camera comment the line above and uncomment this one
		//image->Write("/CameraImage.bmp");
		//visionScores->PutBoolean("image gotten", true);
		BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
		//thresholdImage->Write("/threshold.bmp");
		//visionScores->PutBoolean("threshold computed", true);
		BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
		//convexHullImage->Write("/ConvexHull.bmp");
		//visionScores->PutBoolean("convex hull", true);
		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1);	//Remove small particles
		//filteredImage->Write("Filtered.bmp");
		//visionScores->PutBoolean("filtered computed", true);
		vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle
		scores = new Scores[reports->size()];
		//visionScores->PutBoolean("Image analyzed?", true);
	
		//Iterate through each particle, scoring it and determining whether it is a target or not
		isHighGoal=false;
		isMidGoal=false;
		for (unsigned i = 0; i < reports->size(); i++) {
			ParticleAnalysisReport *report = &(reports->at(i));
			
			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage, report, true);
			scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, false);			
			scores[i].xEdge = scoreXEdge(thresholdImage, report);
			scores[i].yEdge = scoreYEdge(thresholdImage, report);
			
			if(scoreCompare(scores[i], false))
			{
				//printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
				//printf("Distance: %f \n", computeDistance(thresholdImage, report, false));
				isHighGoal=true;
				
				highX=report->center_mass_x;
				highY=report->center_mass_y;
				highWidth=report->boundingRect.width;
				highHeight=report->boundingRect.height;
				highCenterXNormal=report->center_mass_x_normalized;
				highCenterYNormal=report->center_mass_y_normalized;
				highDistance=computeDistance(thresholdImage, report, false);
				highDistance2=computeDistance2(thresholdImage, report);
			} else if (scoreCompare(scores[i], true)) {
				//printf("particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
				//printf("Distance: %f \n", computeDistance(thresholdImage, report, true));
				isMidGoal=true;
				midCenterX=report->center_mass_x_normalized;
				midCenterY=report->center_mass_y_normalized;
				midDistance=computeDistance(thresholdImage, report, true);
				midDistance2=computeDistance2(thresholdImage, report);
			} else {
				//printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
			}
			
		
			//printf("rect: %f  ARinner: %f \n", scores[i].rectangularity, scores[i].aspectRatioInner);
			//printf("ARouter: %f  xEdge: %f  yEdge: %f  \n", scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);	
		}
		//printf("\n");
		// be sure to delete images after using them
		delete filteredImage;
		delete convexHullImage;
		delete thresholdImage;
		delete image;
		
		//delete allocated reports and Scores objects also
		delete scores;
		delete reports;
		
		updated = true;

		Wait(delay);
	}
	camera.DeleteInstance();
	return 0;
}
	
 int start_cpp_task(UINT32 obj)
 {
 	Vision2823 *t = (Vision2823 *) obj;
 	//printf("Jeremy says sizeof obj %d, sizeof UINT32 %d\n", sizeof(t), sizeof(obj));
 	return t->Run();
 }
 void Vision2823::Start(void)
 {
	 running = true;
	 task->Start((UINT32) this);
 }
 
 void Vision2823::Stop(void)
 {
	 running = false;
	 task->Stop();
 }
