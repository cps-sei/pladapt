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

#include <pladapt/SDPRAAdaptationManager.h>
#include <map>
#include <iostream>
#include <stdexcept>
#include <float.h>
#include <memory>

#define EXTRACT_POLICY 1
#define PRINT_POLICY 0
#define DUMP_REACHABILITY 0

// if 1, it uses the minimum survivability (given the worst environment)
#define WORST_CASE 0

using namespace std;

namespace pladapt {

const char* SDPRAAdaptationManager::PROBABILITY_BOUND = "probabilityBound";


void SDPRAAdaptationManager::initialize(std::shared_ptr<const ConfigurationManager> configMgr, const YAML::Node& params) {
	SDPAdaptationManager::initialize(configMgr, params);

	if (params[PROBABILITY_BOUND].IsDefined()) {
		survivalRequirement = params[PROBABILITY_BOUND].as<double>();
	}
}

TacticList SDPRAAdaptationManager::evaluate(const Configuration& currentConfigObj, const EnvironmentDTMCPartitioned& envDTMC,
        const UtilityFunction& utilityFunction, unsigned horizon) {

	if (horizon == 0) {
		throw std::invalid_argument("SDPAdaptationManager::evaluate() called with horizon = 0");
	}

	/* check if we need to adjust the horizon to the environment size */
	if ((envDTMC.getNumberOfParts() - 1) < horizon) {
		if (envDTMC.getNumberOfParts() > 1 && envDTMC.isLastPartFinal()) {
			horizon = envDTMC.getNumberOfParts() - 1;
			cout << "warning: environment is shorter than horizon" << endl;
		}
	}

    const ConfigurationSpace& configSpace = pConfigMgr->getConfigurationSpace();
    unsigned currentConfig = configSpace.getIndex(currentConfigObj);
    if (debug) {
        cout << "current config: " << currentConfigObj << " (" << currentConfig << ')' << endl;
    }

	lastCurrentConfig = currentConfig;
    survivalProbs.clear();
	survivalProbs.resize(configSpace.size(), 0.0);

    typedef std::pair<unsigned, unsigned> SystemEnvPair; // (sysIndex, envIndex)
    typedef map<SystemEnvPair, double> StateUtility;

    auto pUtil = unique_ptr<StateUtility>(new StateUtility);
    auto pNextUtil = unique_ptr<StateUtility>();

    auto pSurvive = unique_ptr<StateUtility>(new StateUtility);
    auto pNextSurvive = unique_ptr<StateUtility>();


    /*
     * Since the state transition determines the action, the optimal policy
     * is recorded as the state that should follow a state at time t
     *
     * Note: we don't really need the whole policy because the best state at time 0
     * determines the tactics that must be started.
     */
#if EXTRACT_POLICY
    unsigned policy[horizon][configSpace.size()];
#endif


    // compute utility at the horizon
    unsigned t = horizon;
    for (unsigned s = 0; s < configSpace.size(); s++) {
        const Configuration& config = configSpace.getConfiguration(s);

        unsigned partIndex = min(t, envDTMC.getNumberOfParts() - 1);
        const DTMCPartitionedStates::Part& envPart = envDTMC.getPart(partIndex);
        for (DTMCPartitionedStates::Part::const_iterator envState = envPart.begin();
                envState != envPart.end(); envState++) {
                double survive = utilityFunction.getMultiplicativeUtility(config, envDTMC.getStateValue(*envState), t);
                assert(pUtil->find(SystemEnvPair(s, *envState)) == pUtil->end());
                (*pUtil)[SystemEnvPair(s, *envState)] =
                    survive
                    * (utilityFunction.getAdditiveUtility(config, envDTMC.getStateValue(*envState), t)
                           + utilityFunction.getFinalReward(config, envDTMC.getStateValue(*envState), t));
                (*pSurvive)[SystemEnvPair(s, *envState)] = survive;

                if (debug) {
                    cout << "t=" << t << endl;
                    cout << "util(" << config << ", " << envDTMC.getStateValue(*envState) << ")="
                            << (*pUtil)[SystemEnvPair(s, *envState)]
                            << " S=" << (*pSurvive)[SystemEnvPair(s, *envState)] << endl;
                }
        }
    }

    // for t=H..1,0 (in the paper 0 is a separate case)
    while (t > 0) {
        t--;
        if (debug) {
            cout << "==>> t=" << t << endl;
        }

        // make last computed utilities, the next
        pNextUtil = std::move(pUtil);
        pUtil.reset(new StateUtility);

        // make last computed survival prob, the next
        pNextSurvive = std::move(pSurvive);
        pSurvive.reset(new StateUtility);


        // \forall c \in C
        for (unsigned s = 0; s < configSpace.size(); s++) {

            // t = 0 is now before adapting, so only the current config is valid
            if (t == 0 && s != currentConfig) {
                continue;
            }

            // if it isn't reachable now (i.e., not a valid initial state), don't bother
            // note that t=1 is the first decision period. That is, the env at t=1
            // is the env we will have during the period that is about to start,
            // which will be preceded, possibly, by immediate adaptations
            if (t == 1 && !isReachableImmediately(currentConfig, s)) {
                if (debug) {
                    cout << "Not reachable: " << configSpace.getConfiguration(s) << '(' << s << ')' << endl;
                }
                continue;
            }

            if (debug) {
                cout << "Reachable: " << configSpace.getConfiguration(s) << '(' << s << ')' << endl;
            }

            const Configuration& config = configSpace.getConfiguration(s);

            // \forall e \in E_t
            unsigned partIndex = min(t, envDTMC.getNumberOfParts() - 1);
            const DTMCPartitionedStates::Part& envPart = envDTMC.getPart(partIndex);
            for (DTMCPartitionedStates::Part::const_iterator envState = envPart.begin();
                    envState != envPart.end(); envState++) {
#if PLADAPT_SDP_NOPARTITION
                if (t == 0 && *envState != 0) {
                    continue;
                }
#endif
                assert(t > 0 || *envState == 0); // we should only have the root env state at t=0

#if EXTRACT_POLICY
                unsigned bestNextState = 0; // assume this for now
#endif
                double localUtil = 0;
                double multiplicativeUtil = 1.0;
                /*
                 * if t = 0 we're at the current state, so we don't need to
                 * add the local utility.
                 * HOWEVER if we extend this to have rewards due to actions,
                 * we do need to add that here (as well as above)
                 */
                if (t > 0) {
                    localUtil = utilityFunction.getAdditiveUtility(config, envDTMC.getStateValue(*envState), t);
                    multiplicativeUtil = utilityFunction.getMultiplicativeUtility(config, envDTMC.getStateValue(*envState), t);
                }

                double maxUtil = -DBL_MAX;
                double surviveForMax = 0.0;
                double maximumSurvivalProbability = 0.0;
                bool firstReachableState = true;

                // c' \in C^T(c)
                for (unsigned nextS = 0; nextS < configSpace.size(); nextS++) {
                    if ((t == 0 && !isReachableImmediately(s, nextS))
                            || (t > 0 && !isReachableFromConfig(s, nextS))) {
                        continue;
                    }
                    if (debug) {
                    	cout << "nextS=" << nextS << endl;
                    }
                    double util = utilityFunction.getAdaptationReward(config, configSpace.getConfiguration(nextS), t);
#if WORST_CASE
                    double survive = 1.0;
#else
                    double survive = 0;
#endif
                    // \sum_{e' \in E_{t+1}} p(e'|e) S^{t+1}(c',e')
                    unsigned nextPartIndex = min(t + 1, envDTMC.getNumberOfParts() - 1);
                    const DTMCPartitionedStates::Part& nextEnvPart = envDTMC.getPart(nextPartIndex);
                    for (DTMCPartitionedStates::Part::const_iterator nextEnvState = nextEnvPart.begin();
                            nextEnvState != nextEnvPart.end(); nextEnvState++) {
                        SystemEnvPair nextSysEnvPair(nextS, *nextEnvState);
                        //assert(pNextUtil->find(nextSysEnvPair) != pNextUtil->end());
                        try {
                            util += envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                    * pNextUtil->at(nextSysEnvPair);
#if WORST_CASE
                            survive = min(survive, multiplicativeUtil * envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                            * pNextSurvive->at(nextSysEnvPair));
#else
                            if (debug && t==0 && s==8 && nextS==9 ) {
                                cout << "survive += " << multiplicativeUtil << " * " << envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                               << " * " << pNextSurvive->at(nextSysEnvPair)
											   << " pair " << nextSysEnvPair.first << "," << nextSysEnvPair.second
											   << endl;
                            }
                            survive += envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                            * pNextSurvive->at(nextSysEnvPair);
#endif
                        } catch(...) {
                            cout << "didn't find util for (" << configSpace.getConfiguration(nextSysEnvPair.first)
                                    << ',' << envDTMC.getStateValue(nextSysEnvPair.second) << " at t=" << t + 1 << endl;
                            assert(false);
                        }
                    } // \sum_{e' \in E_{t+1}} p(e'|e) S^{t+1}(c',e')


                    // s(c,e) \sum_{e' \in E_{t+1}} p(e'|e) S^{t+1}(c',e')
                    survive *= multiplicativeUtil;

                    if (t==0) {
                    	survivalProbs[nextS] = survive;
                    }

                    if (debug) {
                        cout << "\t*->" << '(' << nextS << ')'
                        		<< " util=" << util
    							<< " survProb=" << survive
                                << endl;
                    }

                    maximumSurvivalProbability = max(maximumSurvivalProbability, survive);

                    if (survive < survivalRequirement) {
                        if (debug) {
                            cout << "\t->" << configSpace.getConfiguration(nextS) << '(' <<nextS << ')'
                                    << " does not meet survival survival=" << survive
                                    << " < " << survivalRequirement << " multiplicativeUtil=" << multiplicativeUtil << endl;
                        }
                        continue;
                    }

                    if (firstReachableState || util >= maxUtil) {

                        // for equal utility, prefer the current configuration
                        if (firstReachableState || nextS == currentConfig || util > maxUtil) {
                            maxUtil = util;
                            surviveForMax = survive;
#if EXTRACT_POLICY
                            bestNextState = nextS;
#endif
                        }
                    }
                    firstReachableState = false;
                } // c' \in C^T(c)

                if (maxUtil > -DBL_MAX) {
                    maxUtil = multiplicativeUtil * (localUtil + maxUtil);
                    if (debug) {
                        cout << "\t->" << configSpace.getConfiguration(bestNextState) << '(' << bestNextState << ')'
                        		<< " util=" << maxUtil << " locUtil=" << localUtil << " multUtil=" << multiplicativeUtil
    							<< " survProb=" << surviveForMax
                                << " env=" << envDTMC.getStateValue(*envState)
                                << endl;
                    }
                } else { // otherwise, C_T^t(c,e) = \emptyset
                	if (debug) {
                		assert(firstReachableState);
						cout << "\t-> no next state meets survivability requirement for env="
								<< envDTMC.getStateValue(*envState) << endl;
					}
                }

                assert(pUtil->find(SystemEnvPair(s, *envState)) == pUtil->end());
                (*pUtil)[SystemEnvPair(s, *envState)] = maxUtil;
#if 0
                (*pSurvive)[SystemEnvPair(s, *envState)] = surviveForMax;
#else
	// fix in new paper (2020)
                (*pSurvive)[SystemEnvPair(s, *envState)] = (surviveForMax > 0.0) ? surviveForMax : maximumSurvivalProbability;
#endif
#if EXTRACT_POLICY
                policy[t][s] = bestNextState;
                if (t == 0 && s == currentConfig && surviveForMax < survivalRequirement) {
                    cout << "Survivability requirement cannot be met "
                            << surviveForMax << '<' << survivalRequirement << endl;
                    return evaluate2(currentConfigObj, envDTMC, utilityFunction, horizon);
                }
#endif
            } // \forall e \in E_t
        } // \forall c \in C
    } // for t=H..1,0 (in the paper 0 is a separate case)

    // now we need to find the best initial state
    unsigned bestInitialState = policy[0][currentConfig];

#if PRINT_POLICY
    stringstream policyStr;
    policyStr << "policy @t0: " << configSpace.getConfiguration(bestInitialState) << ": [";
#endif

    // find the tactics that should be started, if any
    TacticList tactics;
    bool foundEntry = false;
    const ReachabilityRelation::ReachableList& reachable = pImmediateReachabilityRelation->getReachableFrom(currentConfig);
    for (ReachabilityRelation::ReachableList::const_iterator entry = reachable.begin(); entry != reachable.end(); ++entry) {
        if (entry->configIndex == bestInitialState) {
            foundEntry = true;
            tactics = entry->tactics;
#if PRINT_POLICY
            if (entry->tactics.size() > 0) {
                for (const auto& tactic : entry->tactics) {
                    policyStr << ' ' << tactic;
                }
            }
#endif
        }
    }
    assert(foundEntry); // should've found it

#if PRINT_POLICY
    policyStr << "]" << endl;
    for (unsigned t = 1; t < horizon; t++) {
        unsigned configAfterOneStep = pStepReachabilityRelation->getReachableFrom(bestInitialState).front().configIndex;
        unsigned lastState = bestInitialState;
        bestInitialState = policy[t][bestInitialState];
        policyStr << "policy @t" << t << ": " << configSpace.getConfiguration(bestInitialState) << ": [";
        const ReachabilityRelation::ReachableList& reachable = pImmediateReachabilityRelation->getReachableFrom(configAfterOneStep);
        bool found = false;
        for (ReachabilityRelation::ReachableList::const_iterator entry = reachable.begin(); entry != reachable.end(); ++entry) {
            if (entry->configIndex == bestInitialState) {
                found = true;
                for (const auto& tactic : entry->tactics) {
                    policyStr << ' ' << tactic;
                }
                break;
            }
        }
        policyStr << "]" << endl;
        if (!found) {
            cout << "Print policy error: can't find transition from "
                    << configSpace.getConfiguration(lastState)
                    << " to "
                    << configSpace.getConfiguration(bestInitialState)
                    << endl;
        }
    }
    cout << policyStr.str();
#endif

    return tactics;
}


TacticList SDPRAAdaptationManager::evaluate2(const Configuration& currentConfigObj, const EnvironmentDTMCPartitioned& envDTMC,
        const UtilityFunction& utilityFunction, unsigned horizon) {

	/* check if we need to adjust the horizon to the environment size */
	if ((envDTMC.getNumberOfParts() - 1) < horizon) {
		if (envDTMC.getNumberOfParts() > 1 && envDTMC.isLastPartFinal()) {
			horizon = envDTMC.getNumberOfParts() - 1;
			cout << "warning: environment is shorter than horizon" << endl;
		}
	}

    const ConfigurationSpace& configSpace = pConfigMgr->getConfigurationSpace();
    unsigned currentConfig = configSpace.getIndex(currentConfigObj);
    if (debug) {
        cout << "current config: " << currentConfigObj << " (" << currentConfig << ')' << endl;
    }

	lastCurrentConfig = currentConfig;
    survivalProbs.clear();
	survivalProbs.resize(configSpace.size(), 0.0);

    typedef std::pair<unsigned, unsigned> SystemEnvPair; // (sysIndex, envIndex)
    typedef map<SystemEnvPair, double> StateUtility;

    auto pUtil = unique_ptr<StateUtility>(new StateUtility);
    auto pNextUtil = unique_ptr<StateUtility>();

    auto pSurvive = unique_ptr<StateUtility>(new StateUtility);
    auto pNextSurvive = unique_ptr<StateUtility>();


    /*
     * Since the state transition determines the action, the optimal policy
     * is recorded as the state that should follow a state at time t
     *
     * Note: we don't really need the whole policy because the best state at time 0
     * determines the tactics that must be started.
     */
#if EXTRACT_POLICY
    unsigned policy[horizon][configSpace.size()];
#endif


    // compute utility at the horizon
    unsigned t = horizon;
    for (unsigned s = 0; s < configSpace.size(); s++) {
        const Configuration& config = configSpace.getConfiguration(s);

        unsigned partIndex = min(t, envDTMC.getNumberOfParts() - 1);
        const DTMCPartitionedStates::Part& envPart = envDTMC.getPart(partIndex);
        for (DTMCPartitionedStates::Part::const_iterator envState = envPart.begin();
                envState != envPart.end(); envState++) {
                double survive = utilityFunction.getMultiplicativeUtility(config, envDTMC.getStateValue(*envState), t);
                assert(pUtil->find(SystemEnvPair(s, *envState)) == pUtil->end());
                (*pUtil)[SystemEnvPair(s, *envState)] =
                    survive
                    * (utilityFunction.getAdditiveUtility(config, envDTMC.getStateValue(*envState), t)
                           + utilityFunction.getFinalReward(config, envDTMC.getStateValue(*envState), t));
                (*pSurvive)[SystemEnvPair(s, *envState)] = survive;

                if (debug) {
                    cout << "t=" << t << endl;
                    cout << "util(" << config << ", " << envDTMC.getStateValue(*envState) << ")="
                            << (*pUtil)[SystemEnvPair(s, *envState)]
                            << " S=" << (*pSurvive)[SystemEnvPair(s, *envState)] << endl;
                }
        }
    }

    // for t=H..1,0 (in the paper 0 is a separate case)
    while (t > 0) {
        t--;
        if (debug) {
            cout << "==>> t=" << t << endl;
        }

        // make last computed utilities, the next
        pNextUtil = std::move(pUtil);
        pUtil.reset(new StateUtility);

        // make last computed survival prob, the next
        pNextSurvive = std::move(pSurvive);
        pSurvive.reset(new StateUtility);


        // \forall c \in C
        for (unsigned s = 0; s < configSpace.size(); s++) {

            // t = 0 is now before adapting, so only the current config is valid
            if (t == 0 && s != currentConfig) {
                continue;
            }

            // if it isn't reachable now (i.e., not a valid initial state), don't bother
            // note that t=1 is the first decision period. That is, the env at t=1
            // is the env we will have during the period that is about to start,
            // which will be preceded, possibly, by immediate adaptations
            if (t == 1 && !isReachableImmediately(currentConfig, s)) {
                if (debug) {
                    cout << "Not reachable: " << configSpace.getConfiguration(s) << '(' << s << ')' << endl;
                }
                continue;
            }

            if (debug) {
                cout << "Reachable: " << configSpace.getConfiguration(s) << '(' << s << ')' << endl;
            }

            const Configuration& config = configSpace.getConfiguration(s);

            // \forall e \in E_t
            unsigned partIndex = min(t, envDTMC.getNumberOfParts() - 1);
            const DTMCPartitionedStates::Part& envPart = envDTMC.getPart(partIndex);
            for (DTMCPartitionedStates::Part::const_iterator envState = envPart.begin();
                    envState != envPart.end(); envState++) {
#if PLADAPT_SDP_NOPARTITION
                if (t == 0 && *envState != 0) {
                    continue;
                }
#endif
                assert(t > 0 || *envState == 0); // we should only have the root env state at t=0

#if EXTRACT_POLICY
                unsigned bestNextState = 0; // assume this for now
#endif

                double localUtil = 0;
                double multiplicativeUtil = 1.0;
                /*
                 * if t = 0 we're at the current state, so we don't need to
                 * add the local utility.
                 * HOWEVER if we extend this to have rewards due to actions,
                 * we do need to add that here (as well as above)
                 */
                if (t > 0) {
                    localUtil = utilityFunction.getAdditiveUtility(config, envDTMC.getStateValue(*envState), t);
                    multiplicativeUtil = utilityFunction.getMultiplicativeUtility(config, envDTMC.getStateValue(*envState), t);
                }

                double maxUtil = -DBL_MAX;
                double surviveForMax = 0.0;
                double maxSurvive = -1.0; // so that even a 0.0 replaces it
                unsigned maxSurviveConf = 0;
                double maxSurviveUtil = 0.0;

                // c' \in C^T(c)
                for (unsigned nextS = 0; nextS < configSpace.size(); nextS++) {
                    if ((t == 0 && !isReachableImmediately(s, nextS))
                            || (t > 0 && !isReachableFromConfig(s, nextS))) {
                        continue;
                    }
                    if (debug) {
                    	cout << "nextS=" << nextS << endl;
                    }
                    double util = utilityFunction.getAdaptationReward(config, configSpace.getConfiguration(nextS), t);
#if WORST_CASE
                    double survive = 1.0;
#else
                    double survive = 0;
#endif
                    // \sum_{e' \in E_{t+1}} p(e'|e) S^{t+1}(c',e')
                    unsigned nextPartIndex = min(t + 1, envDTMC.getNumberOfParts() - 1);
                    const DTMCPartitionedStates::Part& nextEnvPart = envDTMC.getPart(nextPartIndex);
                    for (DTMCPartitionedStates::Part::const_iterator nextEnvState = nextEnvPart.begin();
                            nextEnvState != nextEnvPart.end(); nextEnvState++) {
                        SystemEnvPair nextSysEnvPair(nextS, *nextEnvState);
                        //assert(pNextUtil->find(nextSysEnvPair) != pNextUtil->end());
                        try {
                            util += envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                    * pNextUtil->at(nextSysEnvPair);
#if WORST_CASE
                            survive = min(survive, multiplicativeUtil * envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                            * pNextSurvive->at(nextSysEnvPair));
#else
                            survive += envDTMC.getTransitionMatrix()(*envState, *nextEnvState)
                                            * pNextSurvive->at(nextSysEnvPair);
#endif
                        } catch(...) {
                            cout << "didn't find util for (" << configSpace.getConfiguration(nextSysEnvPair.first)
                                    << ',' << envDTMC.getStateValue(nextSysEnvPair.second) << " at t=" << t + 1 << endl;
                            assert(false);
                        }
                    } // \sum_{e' \in E_{t+1}} p(e'|e) S^{t+1}(c',e')


                    // s(c,e) \sum_{e' \in E_{t+1}} p(e'|e) S^{t+1}(c',e')
                    survive *= multiplicativeUtil;

                    if (t==0) {
                    	survivalProbs[nextS] = survive;
                    }

                    if (survive < survivalRequirement) { // this had t > 0 &&
                        if (debug) {
                            cout << "\t->" << configSpace.getConfiguration(nextS) << '(' <<nextS << ')'
                                    << " does not meet survival survival=" << survive
                                    << " < " << survivalRequirement << endl;
                        }

                        if (survive >= maxSurvive) {

                        	// for equal survivability, prefer the highest utility
                        	if (util >= maxSurviveUtil || survive > maxSurvive) {

                        		// for equal survivability and utility prefer current config
                        		if (nextS == currentConfig || util > maxSurviveUtil || survive > maxSurvive) {
									maxSurvive = survive;
									maxSurviveConf = nextS;
									maxSurviveUtil = util;
                        		}
                        	}
                        }
                    } else {
                        if (util >= maxUtil) {

                            // for equal utility, prefer the current configuration
                            if (nextS == currentConfig || util > maxUtil) {
                                maxUtil = util;
                                surviveForMax = survive;
    #if EXTRACT_POLICY
                                bestNextState = nextS;
    #endif
                            }
                        }
                    }
                }

                if (maxUtil == -DBL_MAX) { // none satisfied the survival requirement
                    maxUtil = maxSurviveUtil;
                    surviveForMax = maxSurvive;
#if EXTRACT_POLICY
                   bestNextState = maxSurviveConf;
#endif
                }
                maxUtil = multiplicativeUtil * (localUtil + maxUtil);

                if (debug) {
                    cout << "\t->" << configSpace.getConfiguration(bestNextState) << '(' << bestNextState << ')'
                    		<< " util=" << maxUtil << " locUtil=" << localUtil << " multUtil=" << multiplicativeUtil
							<< " survProb=" << surviveForMax
                            << " env=" << envDTMC.getStateValue(*envState)
                            << endl;
                }

                assert(pUtil->find(SystemEnvPair(s, *envState)) == pUtil->end());
                (*pUtil)[SystemEnvPair(s, *envState)] = maxUtil;
                (*pSurvive)[SystemEnvPair(s, *envState)] = surviveForMax;
#if EXTRACT_POLICY
                policy[t][s] = bestNextState;
                if (t == 0 && s == currentConfig && surviveForMax < survivalRequirement) {
                    cout << "Survivability requirement still cannot be met "
                            << surviveForMax << '<' << survivalRequirement << endl;
                }
#endif
            } // \forall e \in E_t
        } // \forall c \in C
    } // for t=H..1,0 (in the paper 0 is a separate case)

    // now we need to find the best initial state
    unsigned bestInitialState = policy[0][currentConfig];

#if PRINT_POLICY
    stringstream policyStr;
    policyStr << "policy @t0: " << configSpace.getConfiguration(bestInitialState) << ": [";
#endif

    // find the tactics that should be started, if any
    TacticList tactics;
    bool foundEntry = false;
    const ReachabilityRelation::ReachableList& reachable = pImmediateReachabilityRelation->getReachableFrom(currentConfig);
    for (ReachabilityRelation::ReachableList::const_iterator entry = reachable.begin(); entry != reachable.end(); ++entry) {
        if (entry->configIndex == bestInitialState) {
            foundEntry = true;
            tactics = entry->tactics;
#if PRINT_POLICY
            if (entry->tactics.size() > 0) {
                for (const auto& tactic : entry->tactics) {
                    policyStr << ' ' << tactic;
                }
            }
#endif
        }
    }
    assert(foundEntry); // should've found it

#if PRINT_POLICY
    policyStr << "]" << endl;
    for (unsigned t = 1; t < horizon; t++) {
        unsigned configAfterOneStep = pStepReachabilityRelation->getReachableFrom(bestInitialState).front().configIndex;
        unsigned lastState = bestInitialState;
        bestInitialState = policy[t][bestInitialState];
        policyStr << "policy @t" << t << ": " << configSpace.getConfiguration(bestInitialState) << ": [";
        const ReachabilityRelation::ReachableList& reachable = pImmediateReachabilityRelation->getReachableFrom(configAfterOneStep);
        bool found = false;
        for (ReachabilityRelation::ReachableList::const_iterator entry = reachable.begin(); entry != reachable.end(); ++entry) {
            if (entry->configIndex == bestInitialState) {
                found = true;
                for (const auto& tactic : entry->tactics) {
                    policyStr << ' ' << tactic;
                }
                break;
            }
        }
        policyStr << "]" << endl;
        if (!found) {
            cout << "Print policy error: can't find transition from "
                    << configSpace.getConfiguration(lastState)
                    << " to "
                    << configSpace.getConfiguration(bestInitialState)
                    << endl;
        }
    }
    cout << policyStr.str();
#endif

    return tactics;
}


bool SDPRAAdaptationManager::supportsStrategy() const {
	return false;
}

std::shared_ptr<Strategy> SDPRAAdaptationManager::getStrategy() {
	return std::shared_ptr<Strategy>();
}

std::set<SDPRAAdaptationManager::SurvivalInfo> SDPRAAdaptationManager::getSurvivalInfo() const {
	std::set<SurvivalInfo> info;

	// add an entry for each immediately reachable state
	const ReachabilityRelation::ReachableList& reachable = pImmediateReachabilityRelation->getReachableFrom(lastCurrentConfig);
//	cout << "getSurvivalInfo() from " << pConfigMgr->getConfigurationSpace().getConfiguration(lastCurrentConfig)
//			<< " reachables:" << endl;
	for (ReachabilityRelation::ReachableList::const_iterator entry = reachable.begin(); entry != reachable.end(); ++entry) {
//		cout << "to: " << pConfigMgr->getConfigurationSpace().getConfiguration(entry->configIndex)
//				<< " tactics: ";
//		for (const auto& t : entry->tactics) {
//			cout << ' ' << t;
//		}
//		cout << endl;
		SurvivalInfo infoEntry;
		infoEntry.probability = survivalProbs[entry->configIndex];
		infoEntry.tactics = entry->tactics;
		info.insert(infoEntry);
	}
	return info;
}


SDPRAAdaptationManager::~SDPRAAdaptationManager() {
    // TODO Auto-generated destructor stub
}

} /* namespace pladapt */
