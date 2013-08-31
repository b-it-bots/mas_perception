/*
* This file is part of MultiBoost, a multi-class 
* AdaBoost learner/classifier
*
* Copyright (C) 2005-2006 Norman Casagrande
* For informations write to nova77@gmail.com
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
*/

/**
* \file SingleStumpLearner.h A single threshold decision stump learner. 
*/

#ifndef __SINGLE_STUMP_LEARNER_H
#define __SINGLE_STUMP_LEARNER_H

#include "weak_learners/stump_learner.cpp"
#include "utils/args.cpp"
#include "io/input_data.cpp"
#include "io/class_mappings.cpp"

#include <vector>
#include <fstream>
#include <cassert>
#include <limits>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost {

/**
* A \b single threshold decision stump learner. 
* There is ONE and ONE ONLY threshold here.
*/
class SingleStumpLearner : public StumpLearner
{
public:

   /**
   * The destructor. Must be declared (virtual) for the proper destruction of 
   * the object.
   */
   virtual ~SingleStumpLearner() {}

   /**
   * Returns itself as object.
   * @remark It uses the trick described in http://www.parashift.com/c++-faq-lite/serialization.html#faq-36.8
   * for the auto-registering classes.
   * @date 14/11/2005
   */
   virtual BaseLearner* create() { return new SingleStumpLearner(); }

   /**
   * Run the learner to build the classifier on the given data.
   * @param pData The pointer to the data
   * @see BaseLearner::run
   * @date 11/11/2005
   */
   virtual void run(InputData* pData);

   /**
   * Save the current object information needed for classification,
   * that is the single threshold.
   * @param outputStream The stream where the data will be saved
   * @param numTabs The number of tabs before the tag. Useful for indentation
   * @remark To fully save the object it is \b very \b important to call
   * also the super-class method.
   * @see StumpLearner::save()
   * @date 13/11/2005
   */
   virtual void save(ofstream& outputStream, int numTabs = 0);

   /**
   * Load the xml file that contains the serialized information
   * needed for the classification and that belongs to this class.
   * @param st The stream tokenizer that returns tags and values as tokens
   * @see save()
   * @date 13/11/2005
   */
   virtual void load(nor_utils::StreamTokenizer& st);

   /**
   * Returns a vector of double holding any data that the specific weak learner can generate
   * using the given input dataset. Right now just a single case is contemplated, therefore
   * "reason" is not used. The returned vector of data correspond to:
   * \f[
   * \left\{ v^{(t)}_1, v^{(t)}_2, \cdots, v^{(t)}_K, \phi^{(t)}({\bf x}_1), 
   * \phi^{(t)}({\bf x}_2), \cdots, \phi^{(t)}({\bf x}_n) \right\}
   * \f]
   * where \f$v\f$ is the alignment vector, \f$\phi\f$ is the discriminative function and
   * \f$t\f$ is the current iteration (to which this instantiation of weak learner belongs to).
   * @param data The vector with the returned data.
   * @param pData The pointer to the data points (the examples) previously loaded.
   * @remark This particular method has been created for analysis purposes (research). If you
   * need to get similar analytical information from this weak learner, add it to the function
   * and uncomment the parameter "reason".
   * @see BaseLearner::getStateData
   * @see Classifier::saveSingleStumpFeatureData
   * @date 10/2/2006
   */
   virtual void getStateData( vector<double>& data, const string& /*reason = ""*/, InputData* pData = 0 );

protected:

   /**
   * A discriminative function. 
   * @remarks Positive or negative do NOT refer to positive or negative classification.
   * This function is equivalent to the phi function in my thesis.
   * @param val The value to discriminate
   * @return +1 if \a val is on one side of the border for \a classIdx and -1 otherwise
   * @date 11/11/2005
   * @see classify
   */
   virtual char phi(double val, int /*classIdx*/);

   /**
   * Find the threshold for column \a columnIndex.
   * @param dataBegin The iterator to the beginning of the data.
   * @param dataEnd The iterator to the end of the data.
   * @param pData The pointer to the data.
   * @param threshold The threshold to update.
   * @param mu The The class-wise rates to update.
   * @param v The alignment vector to update.
   * @see StumpLearner::sRates
   * @see run
   * @warning Cannot be virtual because it is a member function template.
   * @remark I cannot define a typedef for vector< pair<int, T> >::iterator because
   * C++ does not support (yet?) template typenames.
   * @date 11/11/2005
   */
   template <typename T>
   void findThreshold(const typename vector< pair<int, T> >::iterator& dataBegin,
                      const typename vector< pair<int, T> >::iterator& dataEnd,
                      InputData* pData, double& threshold,
                      vector<sRates>& mu, vector<double>& v);

   double _threshold; //!< the single threshold of the decision stump
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// The implementation of the template function findThreshold of class
// SingleStumpLearner
template <typename T>
void SingleStumpLearner::findThreshold(const typename vector< pair<int, T> >::iterator& dataBegin,
                                       const typename vector< pair<int, T> >::iterator& dataEnd,
                                       InputData* pData, double& threshold,
                                       vector<sRates>& mu, vector<double>& v)
{
   const int numClasses = ClassMappings::getNumClasses();

   // set to 0
   fill(_leftErrors.begin(), _leftErrors.end(), 0);
   fill(_weightsPerClass.begin(), _weightsPerClass.end(), 0);

   typename vector< pair<int, T> >::iterator currentSplitPos; // the iterator of the currently examined example
   typename vector< pair<int, T> >::iterator previousSplitPos; // the iterator of the example before the current example
   typename vector< pair<int, T> >::const_iterator endArray; // the iterator on the last example (just before dataEnd)

   //////////////////////////////////////////////////
   // Initialization of the class-wise error

   // The class-wise error on the right side of the threshold
   double tmpRightError;

   for (int l = 0; l < numClasses; ++l)
   {
      tmpRightError = 0;

      for( currentSplitPos = dataBegin; currentSplitPos != dataEnd; ++currentSplitPos)
      {
         double weight = pData->getWeight(currentSplitPos->first, l);

         // We assume that class "currClass" is always on the right side;
         // therefore, all points l that are not currClass (x) on right side,
         // are considered error.
         // <l x l x x x l x x> = 3 (if each example has weight 1)
         // ^-- tmpError: error if we set the cut at the extreme left side
         if ( pData->getClass(currentSplitPos->first) != l )
            tmpRightError += weight;

         _weightsPerClass[l] += weight;
      }

      _halfWeightsPerClass[l] = _weightsPerClass[l] / 2;

      assert(tmpRightError < 1);
      _rightErrors[l] = tmpRightError; // store the class-wise error
   }

   ////////////////////////////////////////////////////

   currentSplitPos = dataBegin; // reset position
   endArray = dataEnd;
   --endArray;

   double tmpError = 0;
   double currError = 0;
   double bestError = numeric_limits<double>::max();

   // find the best threshold (cutting point)
   while (currentSplitPos != endArray)
   {
      // at the first split we have
      // first split: x | x x x x x x x x ..
      //    previous -^   ^- current
      previousSplitPos = currentSplitPos;
      ++currentSplitPos; 

      // point at the same position: to skip because we cannot find a cutting point here!
      while ( previousSplitPos->second == currentSplitPos->second && currentSplitPos != endArray)
      {
         for (int l = 0; l < numClasses; ++l)
         { 
            if ( pData->getClass(previousSplitPos->first) == l )
               _leftErrors[l] += pData->getWeight(previousSplitPos->first, l);
            else
               _rightErrors[l] -= pData->getWeight(previousSplitPos->first, l);
         }

         previousSplitPos = currentSplitPos;
         ++currentSplitPos; 
      }

      currError = 0;

      for (int l = 0; l < numClasses; ++l)
      { 
         if ( pData->getClass(previousSplitPos->first) == l )
         {
            // c=current class, x=other class
            // .. c | x x c x c x .. 
            _leftErrors[l] += pData->getWeight(previousSplitPos->first, l);
         }
         else
         {
            // c=current class, x=other class
            // .. x | x x c x c x ..
            _rightErrors[l] -= pData->getWeight(previousSplitPos->first, l);
         }

         tmpError = _rightErrors[l] + _leftErrors[l];

         // switch the class-wise error if it is bigger than chance
         if(tmpError > _halfWeightsPerClass[l] + _smallVal)
            tmpError = _weightsPerClass[l] - tmpError;

         currError += tmpError;
         assert(tmpError <= 0.5); // The summed error MUST be smaller than chance
      }

      // the overall error is smaller!
      if (currError < bestError + _smallVal)
      {
         bestError = currError;
         // compute the threshold
         threshold = static_cast<double>( previousSplitPos->second + currentSplitPos->second ) / 2;

         for (int l = 0; l < numClasses; ++l)
         { 
            _bestErrors[l] = _rightErrors[l] + _leftErrors[l];  

            // If we assume that class [l] is always on the right side,
            // here we must flip, as the lowest error is on the left side.
            // example:
            // c=current class, x=other class
            // .. c c c x | c x x x .. = 2 errors (if we flip!)
            if (_bestErrors[l] > _halfWeightsPerClass[l] + _smallVal)
            {
               // In the binary case would be (1-error)
               _bestErrors[l] = _weightsPerClass[l] - _bestErrors[l];
               v[l] = -1;
            }
            else
               v[l] = +1;
         }

      }

   }

   ////////////////////////////////////////////////////

   // Fill the mus. This could have been done in the threshold loop, 
   // but here is done just once
   for (int l = 0; l < numClasses; ++l)
   {
      mu[l].classIdx = l;

      mu[l].rPls  = _weightsPerClass[l]-_bestErrors[l];
      mu[l].rMin  = _bestErrors[l];
      mu[l].rZero = mu[l].rPls + mu[l].rMin;
   }

} // end of findThreshold

} // end of namespace MultiBoost

#endif // __SINGLE_STUMP_LEARNER_H
