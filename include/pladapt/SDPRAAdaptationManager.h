/*******************************************************************************
 * PLA Adaptation Manager
 *
 * Copyright 2017 Carnegie Mellon University. All Rights Reserved.
 * 
 * NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 * INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 * UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS
 * TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE
 * OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE
 * MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND
 * WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 *
 * Released under a BSD-style license, please see license.txt or contact
 * permission@sei.cmu.edu for full terms.
 *
 * [DISTRIBUTION STATEMENT A] This material has been approved for public release
 * and unlimited distribution. Please see Copyright notice for non-US Government
 * use and distribution.
 ******************************************************************************/

#ifndef SDPRAADAPTATIONMANAGER_H_
#define SDPRAADAPTATIONMANAGER_H_

#include <pladapt/SDPAdaptationManager.h>
#include <vector>

namespace pladapt {

class SDPRAAdaptationManager: public SDPAdaptationManager {
public:
    static const char* PROBABILITY_BOUND;

    void initialize(std::shared_ptr<const ConfigurationManager> configMgr, const YAML::Node& params);

    virtual TacticList evaluate(const Configuration& currentConfigObj, const EnvironmentDTMCPartitioned& envDTMC,
            const UtilityFunction& utilityFunction, unsigned horizon);

    virtual bool supportsStrategy() const;
    virtual std::shared_ptr<Strategy> getStrategy();

    struct SurvivalInfo {
    	TacticList tactics;
    	double probability;
    	bool operator<(const SurvivalInfo& other) const {
    		return tactics < other.tactics;
    	}
    };

    std::set<SurvivalInfo> getSurvivalInfo() const;

    virtual ~SDPRAAdaptationManager();

protected:
    double survivalRequirement = 1.0;

    unsigned lastCurrentConfig; // currentConfig in last call to evaluate
    std::vector<double> survivalProbs;

    virtual TacticList evaluate2(const Configuration& currentConfigObj, const EnvironmentDTMCPartitioned& envDTMC,
            const UtilityFunction& utilityFunction, unsigned horizon);
};

} /* namespace pladapt */

#endif /* SDPRAADAPTATIONMANAGER_H_ */
