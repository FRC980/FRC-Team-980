#include "iostream.h"
#include "WPILib.h"
#include "Vision/AxisCamera.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "DigitalOutput.h"
#include <math.h>

#ifndef _CYCLOPS_H_
#define _CYCLOPS_H_

#define GPIO_DISPLAY_PIN_NUM 8

typedef enum {
    TARGET_ALIGNED,
    TARGET_LEFT,
    TARGET_RIGHT,
    TARGET_UNKNOWN
} TargetAlignment;

class Cyclops {

    public:

	Cyclops(void);
	~Cyclops(void);

	void Start(void);
	void Stop(void);
	
	/*
	 * Warning: public functions that access member data
	 *          must be read only.  If this restriction is
	 *          broken semaphores must be added to cyclops so
	 *          that he doesn't poke himself in the eye and
	 *          go blind.
	 */
	unsigned int GetDistanceToTarget(void);
	TargetAlignment IsTargetAligned(void);

    private:

	vector<vector<int> > GetTargetCenters(void);
	void UpdateDistance(int width);
	void UpdateTargetAligned(vector<vector<int> > &points);
	void SendDistance(void);

	AxisCamera& m_camera;
	unsigned int m_latestDistance;
	TargetAlignment m_targetAligned;
	Task *m_cyclopsTask;
	DigitalOutput *m_Out[GPIO_DISPLAY_PIN_NUM];

	friend int targetDiskTask(UINT32 arg);
};

#endif
