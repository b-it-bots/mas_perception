/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "active_tie/sony_dfwvl500_values.h"

namespace atie {
  SonyDFWVL500Values::SonyDFWVL500Values() : ActiveCameraValues() {
    addParameterDefinition("zoom", 40, 1432, 8 * 1e6);
    addParameterDefinition("focus", 0, 447, 2 * 1e6);
    addParameterDefinition("iris", 1597, 4097, 1 * 1e6);
    addParameterDefinition("brightness", 0, 255, 1 * 1e6);
  }
}
