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
 * \file BaseLearner.h The abstract basic (weak) learner.
 */

#ifndef __BASE_LEARNER_H
#define __BASE_LEARNER_H

#include <algorithm>
#include <vector>

#include "io/input_data.cpp"
#include "utils/stream_tokenizer.h"

#include "utils/args.cpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

	/**
	 * Generic base learner.
	 * All the weak learners used by AdaBoost should inherit from this one.
	 * \todo Add a getAlpha for non-binary (ternary) base-classifiers, using line-search.
	 */
	class BaseLearner
	{

 	private:

		//////////////////////////////////////////////////////////////////////////

		/**
		 * Holds the information about the registered learners. Works pretty
		 * much like a class factory.
		 * @see REGISTER_LEARNER
		 * @see RegisteredLearners()
		 * @see create()
		 * @date 21/11/2005
		 */
		class LearnersRegs
		{
 		public:

			/**
			 * Register a weak learner.
			 * @param learnerName The name of the learner
			 * @param pLearnerToRegister The allocated learner to register.
			 * @warning To be used only with macro REGISTER_LEARNER()!
			 * @date 21/11/2005
			 */
			void addLearner(const string& learnerName, BaseLearner* pLearnerToRegister)
			{
				_learners[learnerName] = pLearnerToRegister;
			}

			/**
			 * Check if a given learner has been registered.
			 * @param learnerName The name of the learner.
			 * @date 21/11/2005
			 */
			bool hasLearner(const string& learnerName)
			{
				return (_learners.find(learnerName) != _learners.end());
			}

			/**
			 * Return the allocated learner object.
			 * @param learnerName The name of the learner.
			 * @date 21/11/2005
			 */
			BaseLearner* getLearner(const string& learnerName)
			{
				return _learners[learnerName];
			}

			/**
			 * Return the list of the learners currently registered.
			 * @param learnersList The list of the learners that will be filled.
			 * @date 21/11/2005
			 */
			void getList(vector<string>& learnersList)
			{
				learnersList.clear();
				learnersList.reserve(_learners.size());
				map<string, BaseLearner*>::const_iterator it;
				for (it = _learners.begin(); it != _learners.end(); ++it)
					learnersList.push_back(it->first);
			}

 		private:
			map<string, BaseLearner*> _learners;  //!< The map of the registered learners.
		};

		//////////////////////////////////////////////////////////////////////////

 	public:

		/**
		 * Map of the registered basic learners.
		 * This data is updated statically just by adding the macro
		 * REGISTER_LEARNER(X) where X is the name of the learner
		 * (which \b must match the class name) in the .cpp
		 * file. Example (in file StumpLearner.cpp):
		 * \code
		 * REGISTER_LEARNER(SingleStumpLearner)
		 * \endcode
		 * It is possible to register a class with a different name of
		 * its class name using REGISTER_LEARNER_NAME(X, Y), where Y
		 * is the custom name. For instance:
		 * \code
		 * REGISTER_LEARNER_NAME(SingleStumpLearner, SStump)
		 * \endcode
		 * @remark Only non-abstract classes must be registered!
		 * @remark To prevent the "static initialization order fiasco"
		 * I am using the trick described in http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.13
		 * @see create()
		 * @see LearnersRegs
		 * @date 14/11/2005
		 */
		static LearnersRegs& RegisteredLearners()
		{
			// Construct On First Use Idiom:
			// Since static local objects are constructed the first time control flows
			// over their declaration (only), the new LearnersRegs() statement will only
			// happen once: the first time RegisteredLearners() is called. Every subsequent
			// call will return the same LearnersRegs object (the one pointed to by ans).
			static LearnersRegs* regLerners = new LearnersRegs();
			return *regLerners;
		}

		/**
		 * The constructor. It initializes _smallVal to 1E-10, and _alpha to 0
		 * @see _alpha
		 * @date 11/11/2005
		 */
		BaseLearner()
				: _smallVal(1E-10),
				  _alpha(0)
		{
			_smoothingVal = _smallVal;
		}

		/**
		 * The destructor. Must be declared (virtual) for the proper destruction of
		 * the object.
		 */
		virtual ~BaseLearner()
		{
		}

		/**
		 * Declare weak-learner-specific arguments.
		 * These arguments will be added to the list of arguments under
		 * the group specific of the weak learner. It is called
		 * automatically in main, when the list of arguments is built up.
		 * Use this method to declare the arguments that belongs to
		 * the weak learner only.
		 * @param args The Args class reference which can be used to declare
		 * additional arguments.
		 * @date 28/11/2005
		 */
		virtual void declareArguments(nor_utils::Args& args) = 0;

		/**
		 * Declare arguments that belongs to all weak-learners.
		 * @remarks This method belongs only to this base class and must not
		 * be extended.
		 * @remarks I cannot use the standard declareArguments method, as it
		 * is called only to instantiated objects, and as this class is abstract
		 * I cannot do it.
		 * @param args The Args class reference which can be used to declare
		 * additional arguments.
		 * @date 10/2/2006
		 */
		static void declareBaseArguments(nor_utils::Args& args);

		/**
		 * Set the arguments of the algorithm using the standard interface
		 * of the arguments. Call this to set the arguments asked by the user.
		 * @remark At this level the method does nothing. It is overridden (if necessary)
		 * in the derived classes.
		 * @date 14/11/2005
		 */
		virtual void initOptions(nor_utils::Args& /*args*/)
		{
		}

		/**
		 * Returns a new object of the derived type.
		 * For instance the overriding of this method in SingleStumpLearner
		 * will be:
		 * \code
		 * return new SingleStumpLearner();
		 * \endcode
		 * For that reason every learner must have an empty constructor. Use
		 * setArguments() if you must define some parameters of the learner.
		 * @remark It uses the trick described in http://www.parashift.com/c++-faq-lite/serialization.html#faq-36.8
		 * for the auto-registering classes.
		 * @see SingleStumpLearner::create()
		 * @date 14/11/2005
		 */
		virtual BaseLearner* create() = 0;

		/**
		 * Creates an InputData object that it is good for the
		 * weak learner. Override it if the weak learner
		 * requires another type of data to be loaded
		 * (which must be an extension of InputData).
		 * @warning The object \b must be destroyed by the caller.
		 * @see InputData
		 * @date 21/11/2005
		 */
		virtual InputData* createInputData();

		/**
		 * Run the learner to build the classifier on the given data.
		 * @param pData The pointer to the data.
		 * @warning This function \b must update _alpha too! You can use the
		 * helper functions (the getAlpha with parameters) to update it.
		 * @see getAlpha(double)
		 * @see getAlpha(double, double)
		 * @see getAlpha(double, double, double, double)
		 */
		virtual void run(InputData* pData) = 0;

		/**
		 * Classify the data on the given example index and class using the learned classifier.
		 * @param pData The pointer to the data.
		 * @param idx The index of the example to classify.
		 * @param classIdx The index of the class.
		 * @remark Passing the data and the index to the example is not nice at all.
		 * This will soon be replaced with the passing of the example itself in some
		 * form (probably a structure to the example).
		 * @return A positive value if the classifier thinks that \a val belongs to class
		 * \a classIdx, a negative if it does not. A real value bonded between -1 and 1 is
		 * returned for real AdaBoost (not implemented yet!).
		 */
		virtual double classify(InputData* pData, int idx, int classIdx) = 0;

		/**
		 * Get the value of alpha. This \b must be computed by the algorithm in run()!
		 * @return The value of alpha.
		 * @remark You can use one of the helper function listed in See also to
		 * update _alpha.
		 * @date 11/11/2005
		 * @see getAlpha(double)
		 * @see getAlpha(double, double, double)
		 * @see getAlpha(double, double, double, double)
		 */
		const double getAlpha() const
		{
			return _alpha;
		}

		/**
		 * Save general parameters of the learner, which does not need to be output for
		 * every weak learner (every iteration). For instance, in case of Neural Networks,
		 * the number of layers.
		 * @see loadGeneral
		 * @date 20/12/2005
		 */
		virtual void saveGeneral(ofstream& /*outputStream*/, int /*numTabs = 0*/)
		{
		}

		/**
		 * Load general parameters of the learner, which does not need to be output for
		 * every weak learner (every iteration). For instance, in case of Neural Networks,
		 * the number of layers.
		 * @see saveGeneral
		 * @date 20/12/2005
		 */
		virtual void loadGeneral(nor_utils::StreamTokenizer& /*st*/)
		{
		}

		/**
		 * Serialize the object. The object information needed for classification
		 * will be saved in xml format. This method should be overridden by the derived
		 * classes which will call the superclass first and then serialize their data.
		 * @param outputStream The stream where the data will be saved.
		 * @param numTabs The number of tabs before the tag. Useful for indentation.
		 * @remark At this level only _alpha is saved.
		 * @see load
		 * @date 13/11/2005
		 */
		virtual void save(ofstream& outputStream, int numTabs = 0);

		/**
		 * Unserialize the object. This method will load the information
		 * needed for the classification from the xml file loaded in a
		 * StreamTokenizer class.
		 * @param st The stream tokenizer that returns tags and values as tokens.
		 * @see save
		 * @date 13/11/2005
		 */
		virtual void load(nor_utils::StreamTokenizer& st);

		/**
		 * Returns a vector of double holding any data that the specific weak learner can generate
		 * using the given input dataset. This kind of information belongs only to the logic
		 * of the weak learner, and is therefore implemented into it.
		 * @remark This particular function is a form of compromise that arise with the question:
		 * "how can I generate weak-learner specific data for analysis purposes?". This
		 * almost-generic method was created to solve this issue, even it is not the definitive answer.
		 * The user can implement any behavior overriding this method, and if one or more behavior are
		 * needed, he can specify which one using the parameter \a reason.
		 * @remark I don't like very much the fact that the returned information is limited to
		 * a vector of doubles. This is likely to change. I've been thinking about a
		 * "templetized" parameter, but I don't like the idea of putting this kind of function
		 * into a header. The other possibility is to pass a void pointer, but I don't like that
		 * either as void pointers are evil. But sometimes a little bit of evil is necessary.
		 * @see SingleStumpLearner::getStateData
		 * @see Classifier::saveSingleStumpFeatureData
		 * @date 10/2/2006
		 */
		virtual void getStateData(vector<double>& /*data*/, const string& /*reason = ""*/, InputData* /*pData = 0*/)
		{
		}

 	protected:

		/**
		 * Set the smoothing value for alpha.
		 * It is used with the formula to compute alpha without regularization.
		 * To avoid smoothing following the paper
		 * "Improved Boosting Algorithms using Confidence-rated Predictions",
		 * page 11
		 * (http://www.cs.princeton.edu/~schapire/uncompress-papers.cgi/SchapireSi98.ps)
		 * the value should be set to 1/n, where n is the number of examples.
		 * @param smoothingVal The new smoothing value.
		 * @see getAlpha(double, double)
		 * @date 22/11/2005
		 */
		virtual void setSmoothingVal(double smoothingVal)
		{
			_smoothingVal = smoothingVal;
		}

		/**
		 * Compute alpha using the error.
		 *
		 * A helper function to compute the alpha for the basic AdaBoost with no abstention
		 * and no theta.
		 * The formula is:
		 * \f[
		 * \alpha = \frac{1}{2} \log  \left( \frac{1-error}{error} \right)
		 * \f]
		 * @param error The error of the weak learner.
		 * @remark Use this function to update \a _alpha.
		 * @see _alpha
		 * @date 11/11/2005
		 */
		double getAlpha(double error);

		/**
		 * Compute alpha with abstention.
		 *
		 * A helper function to compute the alpha for AdaBoost with abstention but
		 * no theta.
		 * The formula is:
		 * \f[
		 * \alpha = \frac{1}{2} \log  \left( \frac{\epsilon_+ + \delta}{\epsilon_- + \delta} \right)
		 * \f]
		 * where \f$\delta\f$ is a smoothing value to avoid the zero on the denominator
		 * in case of no error (\a eps_min == 0).
		 * @param eps_min The error rate of the weak learner.
		 * @param eps_pls The correct rate of the weak learner.
		 * @remark Use this function to update \a _alpha.
		 * @remark \a eps_min + \a eps_pls + \a eps_zero = 0!
		 * @see _alpha
		 * @see setSmoothingVal
		 * @date 11/11/2005
		 */
		double getAlpha(double eps_min, double eps_pls);

		/**
		 * Compute alpha with abstention and theta.
		 *
		 * A helper function to compute the alpha for AdaBoost with abstention and
		 * with theta.
		 * The formula is:
		 * \f[
		 * \alpha =
		 * \begin{cases}
		 *     \ln\left( - \frac{\theta\epsilon^{(t)}_{0}}{2(1+\theta)\epsilon^{(t)}_{-}}
		 *     +
		 *     \sqrt{\left(\frac{\theta\epsilon^{(t)}_{0}}{2(1+\theta)\epsilon^{(t)}_{-}}\right)^2
		 *     + \frac{(1 - \theta)\epsilon^{(t)}_{+}}{(1 +
		 *     \theta)\epsilon^{(t)}_{-}}}\right)
		 *     & \mbox{ if } \epsilon^{(t)}_{-} > 0,\\
   *     \ln\left( \frac{(1-\theta)\epsilon^{(t)}_{+}}{\theta\epsilon^{(t)}_{0}}\right)
		 *     & \mbox{ if } \epsilon^{(t)}_{-} = 0.
		 * \end{cases}
		 * \f]
		 * @param eps_min The error rate of the weak learner.
		 * @param eps_pls The correct rate of the weak learner.
		 * @param theta The value of theta.
		 * @remark Use this function to update \a _alpha.
		 * @remark \a eps_min + \a eps_pls + \a eps_zero = 0!
		 * @see _alpha
		 * @date 11/11/2005
		 */
		double getAlpha(double eps_min, double eps_pls, double theta);

		/**
		 * The smoothing value for alpha.
		 * @see setSmoothingVal
		 * @date 22/11/2005
		 */
		double _smoothingVal;

		double _alpha;  //!< The confidence in the current learner.
		const double _smallVal;  //!< A small value.

 	private:
		/**
		 * Fake assignment operator to avoid warning.
		 * @date 6/12/2005
		 */
		BaseLearner& operator=(const BaseLearner&)
		{
		}

	};

// ------------------------------------------------------------------------------

}// end of namespace MultiBoost

/**
 * The macro that \b must be declared by all the Derived classes that can be used
 * for classification.
 * @see REGISTER_LEARNER_NAME
 */
#define REGISTER_LEARNER(X) \
struct Register_##X \
        { Register_##X() { BaseLearner::RegisteredLearners().addLearner(#X, new X()); } }; \
        static Register_##X r_##X;

/**
 * Similarly to REGISTER_LEARNER this macro register the derived class, but in
 * this case a name can be specified, that will differ from the class name.
 * @see REGISTER_LEARNER
 */
#define REGISTER_LEARNER_NAME(NAME, X) \
struct Register_##X \
        { Register_##X() { BaseLearner::RegisteredLearners().addLearner(#NAME, new X()); } }; \
        static Register_##X r_##X;

#endif // __BASE_LEARNER_H
