#include "Cyclops.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "DigitalOutput.h"

#define CAMERA_IP_ADDR   "10.9.80.11"
#define THETA		 24.0
#define TARGET_FT	 2.0
#define FIELD_OF_VIEW_PX 320.0
#define MIDPOINT 	 FIELD_OF_VIEW_PX / 2
#define FIELD_OFFSET	 MIDPOINT * 0.05

typedef struct rect_ {
    int x;
    int y;
    int width;
    int height;
} rect_t;

void cmessage(char *fmt, ...)
{
    char message[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(message, 256, fmt, args);
    va_end(args);

    setErrorData(message, strlen(message), 100);
}


Cyclops::Cyclops(void) : m_camera(AxisCamera::GetInstance(CAMERA_IP_ADDR)),
    m_latestDistance(0),
    m_targetAligned(TARGET_UNKNOWN),
    m_angleOffCenter(0.0),
    m_cyclopsTask(NULL)
{
    for (uint8_t i = 1; i <= GPIO_DISPLAY_PIN_NUM; i++) {
    	m_Out[i-1] = new DigitalOutput(1, i);
    }
}
    
Cyclops::~Cyclops(void)
{
    m_camera.DeleteInstance();
    for (uint8_t i = 1; i <= GPIO_DISPLAY_PIN_NUM; i++) {
    	delete m_Out[i-1];
    }
}

float Cyclops::GetDistanceToTarget(void)
{
    return m_latestDistance;
}

float Cyclops::GetAngleOffCenter(void)
{
    return m_angleOffCenter;
}

void Cyclops::ProcessImage(void)
{
    vector<ParticleAnalysisReport> *reports;
    ParticleAnalysisReport rect;
    ColorImage *image;
    BinaryImage *thresholdImage;
    BinaryImage *bigObjectsImage;
    BinaryImage *convexHullImage;
    BinaryImage *filteredImage;

    reports = NULL;

    if (m_camera.IsFreshImage()) {

        Threshold threshold(92,139,76,255,90,255);
        ParticleFilterCriteria2 criteria[] = {
            {IMAQ_MT_BOUNDING_RECT_WIDTH, 20, 400, false, false},
            {IMAQ_MT_BOUNDING_RECT_HEIGHT, 18, 400, false, false}
        };

        image = m_camera.GetImage();
        thresholdImage = image->ThresholdHSL(threshold);
        bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 1);
        convexHullImage = bigObjectsImage->ConvexHull(false);
        filteredImage = convexHullImage->ParticleFilter(criteria, 2);
        reports = filteredImage->GetOrderedParticleAnalysisReports();

        if(reports->size() == 0) {
            cmessage("No targets");
        } else {
            cmessage("%d targets", reports->size());

            rect = GetHighestTarget(reports);

            UpdateDistance(rect.boundingRect.width);
            UpdateAngleOffCenter(rect);
            UpdateTargetAligned();
        }

        delete filteredImage;
        delete bigObjectsImage;
        delete thresholdImage;
        delete image;
        delete reports;
	    delete convexHullImage;
    } else {
        printf("No fresh image");
    }
}

TargetAlignment Cyclops::IsTargetAligned(void)
{
    return m_targetAligned;
}

#define ARBITRARILY_LARGE_Y 640

ParticleAnalysisReport Cyclops::GetHighestTarget(
                                    vector<ParticleAnalysisReport> *rects)
{
    ParticleAnalysisReport rect;
    int y;
    int x;

    for (unsigned i = 0; i < rects->size(); i++)
    {
        rect = rects->at(i);
        if (rect.boundingRect.top < y) 
        {
	        y = rect.boundingRect.top;
	        x = rect.boundingRect.left;
        }
    }

    return rect;
}

/*
 * We should see four baskets, and the middle two
 * should be in the center of the field of view 
 * plus or minus a 10 degree angle off center.
 */
void Cyclops::UpdateTargetAligned(void)
{
    if (m_angleOffCenter < 2) {
	    m_targetAligned = TARGET_LEFT;
    } else if (m_angleOffCenter > 2) {
    	m_targetAligned = TARGET_RIGHT;
    } else {
	    m_targetAligned = TARGET_ALIGNED;
    }
}

void Cyclops::UpdateDistance(float width)
{
    float FOVft = ((TARGET_FT/width) * FIELD_OF_VIEW_PX)/2.0;

    float distance = FOVft/tan((3.14159*THETA/180.0));

    cmessage("FOVft: %f feet", FOVft);
    cmessage("width: %f feet", width);
    cmessage("distance %f feet", distance);

    m_latestDistance = distance;
}

void Cyclops::UpdateAngleOffCenter(ParticleAnalysisReport &rect)
{
    float delta_px;
    float delta_ft;
    float feet_per_px;

    /*
     * The angle off center is arctan(Feet off center/distance to target)
     *
     * Feet off center = feet per pixel * Delta pixels
     */
    cmessage("rect.boundingRect.width %d", rect.boundingRect.width);

    feet_per_px = 2.0 / (float)rect.boundingRect.width;

    /*
     * Positive result is to the right, negative result
     * is to the left.
     */
    delta_px = (rect.boundingRect.left + 
               (rect.boundingRect.width / 2)) - MIDPOINT;
    delta_ft = feet_per_px * delta_px;
    m_angleOffCenter = atan(delta_ft / m_latestDistance) * 180 / 3.14;

    cmessage("feet_per_px: %f feet\n", feet_per_px);
    cmessage("delta_ft: %f feet\n", delta_ft);
    cmessage("delta_px: %f pixels\n", delta_px);
    cmessage("Angle off center %f angle\n", m_angleOffCenter);
}

void Cyclops::SendDistance(void)
{
	uint8_t upper_nibble;
	uint8_t lower_nibble;
	
	upper_nibble = (int)m_latestDistance / 10;
	lower_nibble = (int)m_latestDistance % 10;
	
	for (int i = 0; i < GPIO_DISPLAY_PIN_NUM/2; i++) {
	    printf("Setting port %d to %d", i, upper_nibble & (1 << i)); 
	    if (upper_nibble & (1 << i)) {
		    m_Out[i]->Set(1);
	    } else {
		    m_Out[i]->Set(0);
	    }
    	    
	    if (lower_nibble & (1 << i)) {
		    m_Out[i+4]->Set(1);
	    } else {
		    m_Out[i+4]->Set(0);
	    }
	}	
}

int targetDiskTask(UINT32 arg)
{
    Timer t;
    Cyclops *cyclops;

    cyclops = (Cyclops *)arg;
    t.Start();

    while (1) 
    {
        t.Reset();
        t.Start();

    	cyclops->ProcessImage();
        cyclops->SendDistance();

        Wait(2);
    }

    return 0;
}

void Cyclops::Start(void)
{
    if (!m_cyclopsTask) {
	    m_cyclopsTask = new Task("distance", (FUNCPTR)targetDiskTask);
	    m_cyclopsTask->Start((UINT32)this);
    }
}

void Cyclops::Stop(void)
{
    if (m_cyclopsTask) {
        m_cyclopsTask->Stop();
        delete m_cyclopsTask;
        m_cyclopsTask = NULL;
    }
}

