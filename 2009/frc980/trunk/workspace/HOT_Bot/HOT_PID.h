#ifndef HOT_PID_H_
#define HOT_PID_H_

/**
 * Class implements a PID Control Loop.
 *
 * On each iteration with new PID control values, call the CalculateMV() method
 *
 */
class HOT_PID
{
  private:
    float m_Kp;                 // proportional gain
    float m_Ki;                 // integral gain
    float m_Kd;                 // derivative gain
    float m_SP_max;             // limits for Set Point
    float m_SP_min;
    float m_PV_max;             // limits for Process Variable
    float m_PV_min;
    float m_MV_max;             // limits for Manipulated Variable
    float m_MV_min;
    float m_error_p;            // last p error term
    float m_error_i;            // last i error term
    float m_error_d;            // last d error term
    float m_tolerance; // on-target tolerance: fraction of m_SP_max, m_SP_min
    float m_enabled;            // enable/disable control

  public:
    HOT_PID(float Kp = 0.0, float Ki = 0.0, float Kd = 0.0,
            float tolerance = 0.0);
    ~HOT_PID(void);

    float GetMV(float SP, float PV);

    void SetGains(float Kp, float Ki, float Kd);

    void SetMVLimits(float MV_max, float MV_min);
    void SetSPLimits(float SP_max, float SP_min);
    void SetPVLimits(float PV_max, float PV_min);

    void SetTolerance(float percent);
    bool OnTarget(void);

    void Enable(void);
    void Disable(void);

    void Reset(void);
};

#endif
