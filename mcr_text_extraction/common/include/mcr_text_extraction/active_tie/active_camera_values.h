/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __ACTIVE_CAMERA_VALUES__
#define __ACTIVE_CAMERA_VALUES__

#include <map>
#include <string>

using namespace std;

namespace atie
{
	class ActiveCameraValues
	{
 	protected:
		map<string, long> parameterFRDelays;
		map<string, int> minValues;
		map<string, int> maxValues;
 	public:
		void addParameterDefinition(string name_, int min_, int max_, long FRDelay_ = 0);
		void observedValue(string name_, int value_);

		long getParameterFRDelay(string name_);
		int translateToSensorValue(string names_, int value_);
		int getRangeSize(string name_);
		int getMin(string name_);
		int getMax(string name_);
	};
}
;

#endif
