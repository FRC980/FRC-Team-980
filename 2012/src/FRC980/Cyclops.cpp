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


Cyclops::Cyclops(void) : m_camera(AxisCamera::GetInstance(CAMERA_IP_ADDR)),
    m_latestDistance(0),
    m_targetAligned(TARGET_UNKNOWN),
    m_cyclopsTask(NULL)
{
    for (uint8_t i = 0; i < GPIO_DISPLAY_PIN_NUM; i++) {
    	m_Out[i] = new DigitalOutput(i);
    }
}
    
Cyclops::~Cyclops(void)
{
    m_camera.DeleteInstance();
    for (uint8_t i = 0; i < GPIO_DISPLAY_PIN_NUM; i++) {
    	delete m_Out[i];
    }
}

void Cyclops::UpdateDistance(int width)
{
    float FOVft = ((TARGET_FT/width) * FIELD_OF_VIEW_PX)/2.0;
    float distance = FOVft/tan((3.14159*THETA/180));

    m_latestDistance = (unsigned int)distance;
}

unsigned int Cyclops::GetDistanceToTarget(void)
{
    return m_latestDistance;
}

vector<vector<int> > Cyclops::GetTargetCenters(void)
{
    vector<vector<int> > points;
    vector<ParticleAnalysisReport> *reports;
    ColorImage *image;
    BinaryImage *thresholdImage;
    BinaryImage *bigObjectsImage;
    BinaryImage *convexHullImage;
    BinaryImage *filteredImage;

    if(m_camera.IsFreshImage())
    {
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

        if(reports->size() == 0)
        {
            printf("No targets");
        }
        else
        {
            for(unsigned i = 0; i < reports->size(); i++)
            {
                ParticleAnalysisReport *r = &(reports->at(i));
                vector<int> temp;
                temp.push_back(r->center_mass_x);
                temp.push_back(r->center_mass_y);
                temp.push_back(r->boundingRect.width);
                points.push_back(temp);
            }
        }
        delete filteredImage;
        delete bigObjectsImage;
        delete thresholdImage;
        delete image;
        delete reports;
	delete convexHullImage;
    }
    else
    {
        printf("No fresh image");
    }
    return points;
}

TargetAlignment Cyclops::IsTargetAligned(void)
{
    return m_targetAligned;
}

/*
 * We should see four baskets, and the middle two
 * should be in the center of the field of view 
 * plus or minus a 10 degree angle off center.
 */
void Cyclops::UpdateTargetAligned(vector<vector<int> > &points)
{
    int x, y;

    x = 0;
    y = 0;

    if (points.size() != 4)
    {
	m_targetAligned = TARGET_UNKNOWN;
    }
	
    for(unsigned i = 0; i < points.size(); i++)
    {
        if (points.at(i).at(1) > y) 
        {
	    y = points.at(i).at(1);
	    x = points.at(i).at(0);
        }
    }
   
    if (x > MIDPOINT - FIELD_OFFSET) 
    {
	m_targetAligned = TARGET_RIGHT;
    } 
    else if (x < MIDPOINT + FIELD_OFFSET)
    {
    	m_targetAligned = TARGET_LEFT;
    } 
    else
    {
	m_targetAligned = TARGET_ALIGNED;
    }
}

void Cyclops::SendDistance(void)
{
	uint8_t upper_nibble;
	uint8_t lower_nibble;
	
	upper_nibble = (m_latestDistance & 0xF0) >> 4;
	lower_nibble = (m_latestDistance & 0x0F);
	
	for (int i = 0; i < GPIO_DISPLAY_PIN_NUM; i++) {
	    printf("Setting port %d to %d", i, m_latestDistance & (1 << i)); 
    	    m_Out[i]->Set(m_latestDistance & (1 << i));
	}	
}

int targetDiskTask(UINT32 arg)
{
    int x, y, width, vertical, horizontal;
    Timer t;
    Cyclops *cyclops;

    cyclops = (Cyclops *)arg;
    t.Start();

    while (1) 
    {
        t.Reset();
        t.Start();
    	vector<vector<int> > points = cyclops->GetTargetCenters();
    
	cyclops->UpdateTargetAligned(points);

    	if (cyclops->IsTargetAligned() == TARGET_ALIGNED) 
    	{
    	    printf("The baskets are aligned");
    	}
    	
	if (points.size() == 0) {
	    printf("There are no rectangles in view");
	}

    	for(unsigned i = 0; i < points.size(); i++)
    	{
	    x = points.at(i).at(0);
	    y = points.at(i).at(1);
	    width = points.at(i).at(2);
	    cyclops->UpdateDistance(width);
	    vertical = y-120;
	    horizontal = x-160;
	    printf("vertical distance from center: %d", vertical);
	    printf("horizontal distance from center: %d", horizontal);
	    printf("width of target: %d", width);
            printf("distance from target: %d", cyclops->GetDistanceToTarget());
    	}
    
	cyclops->SendDistance();

	printf("Time now %f", t.Get());

	Wait(1);
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

