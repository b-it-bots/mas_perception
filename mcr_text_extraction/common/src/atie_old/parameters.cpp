/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "atie_old/parameters.h"


/* Yeah, an ugly global variable, so what?  Since we have no C++
syntax here it might be wiser to compile this as a C file so that we
can use the structure initialization syntax, e.g.  struct

ConfigurableParameters configurableParameters {.<field-name>= <value>, ...};*/
struct ConfigurableParameters configurableParameters= {
  0, 
/* Set to the default values set in parameters.h */
  UNARY_THRESHOLD,
  BINARY_THRESHOLD,
  CHEN_NIBLACK_MAX_RADIUS,
  CHEN_NIBLACK_DEFAULT_K,
  CHEN_NIBLACK_DEFAULT_TD,
  true,
  NULL,
  NULL,
  UNARY_CLASSIFICATION_N_FEATURES,
  ZERNIKE_ORDER,
  PZERNIKE_ORDER,
  false
};

void initParameters() {
  /* Pointer elements within the configurableParameters variable
     should be assigned to the NULL pointer in the initialization of
     configurableParameters and memory should be allocated for them
     within this function with by a call to `realloc'. This will
     ensure that the data is being stored in the heap. By not doing
     so, the behavior of the SWIG wrappers when modifying values is
     not defined and likely to be bogus */
  char normalization[]= MODEL_DATA_INSTALL_PATH"h7-z0-p0.norm";
  char model[]= MODEL_DATA_INSTALL_PATH"h7-z0-p0.svm";

  if (configurableParameters.initialized) return;
  configurableParameters.unarySVMModel= 
    (char*) realloc(configurableParameters.unarySVMModel,
		    sizeof(model));
  configurableParameters.unarySVMNormalization= 
    (char*) realloc(configurableParameters.unarySVMNormalization,
		    sizeof(normalization));
  strcpy(configurableParameters.unarySVMModel, model);
  strcpy(configurableParameters.unarySVMNormalization, normalization);
  configurableParameters.initialized= true;
}

