%module(directors="1") pladapt
%{
#include "pladapt/GenericConfiguration.h"
#include "pladapt/GenericEnvironment.h"
#include "pladapt/GenericUtilityFunction.h"
#include "pladapt/GenericConfigurationManager.h"
#include "pladapt/EnvironmentDTMCPartitioned.h"
#include "pladapt/JavaSDPAdaptationManager.h"
#include "pladapt/GenericEnvironmentDTMCPartitioned.h"
#include "pladapt/timeseries/ScenarioTree.h"
#include "pladapt/timeseries/TimeSeriesPredictor.h"
#include "pladapt/PMCAdaptationManager.h"
#include <sstream>
#include <math.h>
  namespace pladapt {
    double testGeneric(const GenericUtilityFunction& u, const GenericConfiguration& c, const GenericEnvironment& e);
  }
%}

//%javaexception("java.lang.Exception") {
%exception {
  try {
     $action
  } catch (std::exception &e) {
    jclass clazz = jenv->FindClass("java/lang/Exception");
    jenv->ThrowNew(clazz, e.what());
    return $null;
   }
}


%include <std_shared_ptr.i>
%include "std_string.i"

 //%shared_ptr(pladapt::Environment)
%ignore pladapt::Environment::printOn;
%include "pladapt/Environment.h"

%ignore pladapt::GenericProperties;
%include "pladapt/GenericProperties.h"

%ignore pladapt::GenericConfiguration::printOn;
%include "pladapt/GenericConfiguration.h"
%template(getDouble) pladapt::GenericConfiguration::get<double>;
%template(setDouble) pladapt::GenericConfiguration::set<double>;
%template(getInt) pladapt::GenericConfiguration::get<int>;
%template(setInt) pladapt::GenericConfiguration::set<int>;

//%shared_ptr(pladapt::GenericEnvironment)
%ignore pladapt::GenericEnvironment::printOn;
%include "pladapt/GenericEnvironment.h"
%template(getDouble) pladapt::GenericEnvironment::get<double>;
%template(setDouble) pladapt::GenericEnvironment::set<double>;
%template(getInt) pladapt::GenericEnvironment::get<int>;
%template(setInt) pladapt::GenericEnvironment::set<int>;

%shared_ptr(pladapt::GenericConfigurationManager)
%ignore pladapt::GenericConfigurationManager::getConfigurationFromYaml;
%ignore pladapt::GenericConfigurationManager::getConfigurationSpace;
%include "pladapt/GenericConfigurationManager.h"

%feature("director") pladapt::GenericUtilityFunction;
%feature("nodirector") pladapt::GenericUtilityFunction::getAdditiveUtility;
%feature("nodirector") pladapt::GenericUtilityFunction::getMultiplicativeUtility;
%feature("nodirector") pladapt::GenericUtilityFunction::getFinalReward;
%ignore pladapt::GenericUtilityFunction::getAdditiveUtility;
%ignore pladapt::GenericUtilityFunction::getMultiplicativeUtility;
%ignore pladapt::GenericUtilityFunction::getFinalReward;
%include "pladapt/GenericUtilityFunction.h"

 // these methods should not be needed outside of the evaluation
%ignore pladapt::DTMCPartitionedStates::getNumberOfStates;
%ignore pladapt::DTMCPartitionedStates::getTransitionMatrix;
%ignore pladapt::DTMCPartitionedStates::getPart;
%include "pladapt/DTMCPartitionedStates.h"


%ignore pladapt::EnvironmentDTMCPartitioned::printOn;
%ignore pladapt::EnvironmentDTMCPartitioned::getStateValue;
%ignore pladapt::EnvironmentDTMCPartitioned::setStateValue(unsigned state, std::shared_ptr<Environment> pValue);
%include "pladapt/EnvironmentDTMCPartitioned.h"


%include "pladapt/GenericEnvironmentDTMCPartitioned.h"


// add toString methods
namespace pladapt {
  %extend GenericConfiguration {
    std::string toString() {
      std::ostringstream os;
      $self->printOn(os);
      return os.str();
    }
   }
  %extend GenericEnvironment {
    std::string toString() {
      std::ostringstream os;
      $self->printOn(os);
      return os.str();
    }
   }
  %extend EnvironmentDTMCPartitioned {
    std::string toString() {
      std::ostringstream os;
      $self->printOn(os);
      return os.str();
    }
   }
}



%include <std_vector.i>
%template(StringVector) std::vector<std::string>;
%ignore pladapt::AdaptationManager::evaluate(const Configuration& currentConfigObj, const EnvironmentDTMCPartitioned& envDTMC,
    		const UtilityFunction& utilityFunction, unsigned horizon);
%include "pladapt/AdaptationManager.h"

%shared_ptr(pladapt::ConfigurationManager)
%ignore pladapt::SDPAdaptationManager::initialize(std::shared_ptr<const ConfigurationManager> configMgr, const YAML::Node& params);
%ignore pladapt::SDPAdaptationManager::evaluate(const Configuration& currentConfigObj, const EnvironmentDTMCPartitioned& envDTMC,
   const UtilityFunction& utilityFunction, unsigned horizon);
%immutable;
%include "pladapt/SDPAdaptationManager.h"
%mutable;

%include "pladapt/JavaSDPAdaptationManager.h"

namespace pladapt {
  double testGeneric(const GenericUtilityFunction& u, const GenericConfiguration& c, const GenericEnvironment& e);
}

%include "arrays_java.i"
%apply double[] {double *};
%ignore pladapt::timeseries::TimeSeriesPredictor::createScenarioTree;
%include "pladapt/timeseries/TimeSeriesPredictor.h"

namespace pladapt {
  namespace timeseries {
  %extend TimeSeriesPredictor {
    EnvironmentDTMCPartitioned generateEnvironmentDTMC(unsigned branchingDepth, unsigned depth = 0) {
  auto pTree = $self->createScenarioTree(branchingDepth, depth);
  auto envDTMC = pTree->getEnvironmentDTMC(
					   [](double mean) {auto envState = std::make_shared<pladapt::GenericEnvironment>("m"); envState->set<double>("m", mean); envState->set<double>("v", pow(mean, 2)); return envState;}); // assume exponential
      delete pTree;
      return envDTMC;
    }
   }
 }
}

%ignore pladapt::PMCHelper;
%ignore pladapt::PMCAdaptationManager::initialize;
%ignore pladapt::PMCAdaptationManager::evaluate;
%include "pladapt/PMCAdaptationManager.h"
