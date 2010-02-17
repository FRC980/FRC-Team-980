#ifndef UTILS_H
#define UTILS_H

//--- THIS IS DISABLED BECAUSE IT LIVES IN MATH.H
//#define ABS(x)	(((x) > 0) ? (x) : (-x))

//==============================================================================
//! Utilities for use with the robot
/*!\class utils
 *
 * The purpose of the Robot980 class is to contain code specific to the team's
 * robot.
 *    
 */
class utils
{
   private:
   
   public:
      //--- Instance Variables -------------------------------------------------
      
      //--- Constructors -------------------------------------------------------
      /*!\brief The utils contructor
       */
      utils();
      
      //--- Destructors --------------------------------------------------------
      /*!\brief The utils destructor
       */
      virtual ~utils();
      
      //--- Methods ------------------------------------------------------------
      
      /*!\brief A method to limit a particular value
       * \param val A value to limit
       * \param min The minimum value
       * \param max The maximum value
       *
       * This method limits a given value to within the given min
       * and max values.  If the value to limit is outside the limits
       * then the value is set to either the min or max value.
       */
      double limit(double val, double min = -1, double max = 1);
};

#endif // UTILS_H
