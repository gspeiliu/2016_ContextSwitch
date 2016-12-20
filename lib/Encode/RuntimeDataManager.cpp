/*
 * RuntimeDataManager.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: ylc
 */

#include "RuntimeDataManager.h"

#include <llvm/Support/raw_ostream.h>
#include "llvm/IR/Instructions.h"
#include <iterator>
#include <map>
#include <sstream>
#include <utility>
#include <fstream>

using namespace std;
using namespace llvm;

namespace klee {

	RuntimeDataManager::RuntimeDataManager() :
			currentTrace(NULL) {
		traceList.reserve(20);

		allFormulaNum = 0;
		solvingTimes = 0;
		allGlobal = 0;
		brGlobal = 0;
		satBranch = 0;
		unSatBranchBySolve = 0;
		unSatBranchByPreSolve = 0;

		solvingCost = 0.0;
		runningCost = 0.0;
		satCost = 0.0;
		unSatCost = 0.0;


		DTAMCost = 0;
		DTAMSerialCost = 0;
		DTAMParallelCost = 0;
		DTAMhybridCost = 0;
		PTSCost = 0;

		DUInSameThread = 0;
		DUInDiffThread = 0;
		DUFromInit = 0;

		argvOfMain = NULL;
		argcOfMain = 0;
		allMPSet = 0;

		system("rm -rf ./output_info");
		system("mkdir ./output_info");

	}

	RuntimeDataManager::~RuntimeDataManager() {
		for (vector<Trace*>::iterator ti = traceList.begin(), te = traceList.end(); ti != te; ti++) {
			delete *ti;
		}
		string ErrorInfo;
		raw_fd_ostream out_to_file("./output_info/statics.txt", ErrorInfo, sys::fs::F_Append);
		stringstream ss;
		ss << "AllFormulaNum:" << allFormulaNum << "\n";
		ss << "SovingTimes:" << solvingTimes << "\n";
		ss << "TotalNewPath:" << testedTraceList.size() << "\n";
		ss << "TotalOldPath:" << traceList.size() - testedTraceList.size() << "\n";
		ss << "TotalPath:" << traceList.size() << "\n";
		if (testedTraceList.size()) {
			ss << "allGlobal:" << allGlobal * 1.0 / testedTraceList.size() << "\n";
			ss << "brGlobal:" << brGlobal * 1.0 / testedTraceList.size() << "\n";
		} else {
			ss << "allGlobal:0" << "\n";
			ss << "brGlobal:0" << "\n";
		}
		if (testedTraceList.size()) {
			ss << "AllBranch:" << (satBranch + unSatBranchBySolve) * 1.0 / testedTraceList.size() << "\n";
			ss << "satBranch:" << satBranch * 1.0 / testedTraceList.size() << "\n";
		} else {
			ss << "AllBranch:0" << "\n";
			ss << "satBranch:0" << "\n";
		}
		if (satBranch) {
			ss << "satCost:" << satCost / satBranch << "\n";
		} else {
			ss << "satCost:0" << "\n";
		}
		if (testedTraceList.size()) {
			ss << "unSatBranchBySolve:" << unSatBranchBySolve * 1.0 / testedTraceList.size() << "\n";
		} else {
			ss << "unSatBranchBySolve:0" << "\n";
		}
		if (unSatBranchBySolve) {
			ss << "unSatCost:" << unSatCost / unSatBranchBySolve << "\n";
		} else {
			ss << "unSatCost:0" << "\n";
		}
		if (testedTraceList.size()) {
			ss << "unSatBranchByPreSolve:" << unSatBranchByPreSolve * 1.0 / testedTraceList.size() << "\n";
		} else {
			ss << "unSatBranchByPreSolve:0" << "\n";
		}

		ss << "SolvingCost:" << solvingCost << "\n";
		ss << "RunningCost:" << runningCost << "\n";


		ss << "DTAMCost:" << DTAMCost << "\n";
		ss << "DTAMSerialCost:" << DTAMSerialCost << "\n";
		ss << "DTAMParallelCost:" << DTAMParallelCost << "\n";
		ss << "DTAMhybridCost:" << DTAMhybridCost << "\n";
		ss << "PTSCost:" << PTSCost << "\n";

		ss << "DTAMSerialMap:" << allDTAMSerialMap.size() << "\n";
		ss << "DTAMParallelMap:" << allDTAMParallelMap.size() << "\n";
		ss << "DTAMhybridMap:" << allDTAMhybridMap.size() << "\n";
		ss << "taintMap:" << allTaintMap.size() << "\n";

		for (unsigned i = 0; i < allDTAMCost.size(); i++) {

			ss << "\n";
			ss << "times:" << i << "\n";

			ss << "allDTAMCost:" << allDTAMCost[i] << "\n";
			ss << "allDTAMSerialCost:" << allDTAMSerialCost[i] << "\n";
			ss << "allDTAMParallelCost:" << allDTAMParallelCost[i] << "\n";
			ss << "allDTAMhybridCost:" << allDTAMhybridCost[i] << "\n";
			ss << "allPTSCost:" << allPTSCost[i] << "\n";

			ss << "allDTAMSerial:" << DTAMSerial[i] << "\n";
			ss << "allDTAMParallel:" << DTAMParallel[i] << "\n";
			ss << "allDTAMhybrid:" << DTAMhybrid[i] << "\n";

			ss << "allTaint:" << taint[i] << "\n";
			ss << "allTaintPTS:" << taintPTS[i] << "\n";
			ss << "allNoTaintPTS:" << noTaintPTS[i] << "\n";

			ss << "allDTAMSerialMap:" << DTAMSerialMap[i] << "\n";
			ss << "allDTAMParallelMap:" << DTAMParallelMap[i] << "\n";
			ss << "allDTAMhybridMap:" << DTAMhybridMap[i] << "\n";
			ss << "allTaintMap:" << TaintAndPTSMap[i] << "\n";

		}

		ss << "DUInSameThread:" << DUInSameThread << "\n";
		ss << "DUInDiffThread:" << DUInDiffThread << "\n";
		ss << "DUFromInit:" << DUFromInit << "\n";

		ss << "\n";

		out_to_file << ss.str();
		out_to_file.close();
	}

	Trace* RuntimeDataManager::createNewTrace(unsigned traceId) {
		currentTrace = new Trace();
		currentTrace->Id = traceId;
		traceList.push_back(currentTrace);
		return currentTrace;
	}

	Trace* RuntimeDataManager::getCurrentTrace() {
		return currentTrace;
	}

	void RuntimeDataManager::addScheduleSet(Prefix* prefix) {
		scheduleSet.push_back(prefix);
	}

	void RuntimeDataManager::printCurrentTrace(bool file) {
		currentTrace->print(file);
	}

	Prefix* RuntimeDataManager::getNextPrefix() {
		if (scheduleSet.empty()) {
			return NULL;
		} else {
			Prefix* prefix = scheduleSet.front();
			scheduleSet.pop_front();
			return prefix;
		}
	}

	void RuntimeDataManager::clearAllPrefix() {
		scheduleSet.clear();
	}

	bool RuntimeDataManager::isCurrentTraceUntested() {
		bool result = true;
		for (set<Trace*>::iterator ti = testedTraceList.begin(), te = testedTraceList.end(); ti != te; ti++) {
			if (currentTrace->isEqual(*ti)) {
				result = false;
				break;
			}
		}
		currentTrace->isUntested = result;
		if (result) {
			testedTraceList.insert(currentTrace);
		}
		return result;
	}

	void RuntimeDataManager::printAllPrefix(ostream &out) {
		out << "num of prefix: " << scheduleSet.size() << endl;
		unsigned num = 1;
		for (list<Prefix*>::iterator pi = scheduleSet.begin(), pe = scheduleSet.end(); pi != pe; pi++) {
			out << "Prefix " << num << endl;
			(*pi)->print(out);
			num++;
		}
	}

	void RuntimeDataManager::printAllTrace(ostream &out) {
		out << "\nTrace Info:\n";
		out << "num of trace: " << traceList.size() << endl << endl;
		unsigned num = 1;
		for (vector<Trace*>::iterator ti = traceList.begin(), te = traceList.end(); ti != te; ti++) {
			Trace* trace = *ti;
			if (trace->isUntested) {
				out << "Trace " << num << endl;
				if (trace->abstract.empty()) {
					trace->createAbstract();
				}
				for (vector<string>::iterator ai = trace->abstract.begin(), ae = trace->abstract.end(); ai != ae; ai++) {
					out << *ai << endl;
				}
				out << endl;
				num++;
			}
		}
	}

	void RuntimeDataManager::getExplicitDU() {
		vector<Event*>& path = getCurrentTrace()->path;
		std::set<Event*> readSet;
		int pathSize = path.size();

		for (int i = pathSize - 1; i >= 0; i--) {
			if (path[i]->isGlobal) {
				Instruction* I = path[i]->inst->inst;
				if (LoadInst::classof(I)) {
					readSet.insert(path[i]);
				} else if (StoreInst::classof(I)) {
					std::set<Event*>::iterator it = readSet.begin();

					for (; it != readSet.end();) {
						if (path[i]->name == (*it)->name) {
							int asRdLine = (*it)->inst->info->assemblyLine;
							int asWrtLine = path[i]->inst->info->assemblyLine;
							std::pair<int, int> P(asWrtLine, asRdLine);
//							if (DUPair.find(P) == DUPair.end()) {
//								DUPair.insert(P);
								if (path[i]->threadId != (*it)->threadId &&
										diffThreadDUSet.find(P) == diffThreadDUSet.end()) {
									diffThreadDUSet.insert(P);
									DUInDiffThread++;
								} else if (path[i]->threadId == (*it)->threadId &&
										sameThreadDUSet.find(P) == sameThreadDUSet.end()){
									sameThreadDUSet.insert(P);
									DUInSameThread++;
								}
//							}
							readSet.erase(it++);
						} else {
							it++;
						}
					}
				}
			}
		}

		for (std::set<Event*>::iterator it = readSet.begin(),
				ie = readSet.end(); it != ie; it++) {
			int asRdLine = (*it)->inst->info->assemblyLine;
			std::pair<int, int> P(-1, asRdLine);

			if (ReadFromInit.find(P) == ReadFromInit.end()) {
				ReadFromInit.insert(P);
				DUFromInit++;
			}
		}
	}

	void RuntimeDataManager::addcharInputPrefixSet(
			Prefix* prefix, std::vector<std::string> vecStr) {
		charInputPrefixSet.push_back(std::make_pair(prefix, vecStr));
	}

	void RuntimeDataManager::addintInputPrefixSet(
			Prefix* prefix, std::map<std::string, unsigned> mapOfStrInt) {
		intInputPrefixSet.push_back(std::make_pair(prefix, mapOfStrInt));
	}

	std::pair<Prefix*, std::map<std::string, unsigned> > RuntimeDataManager::getNextIntInputPrefix() {
			std::pair<Prefix*, std::map<std::string, unsigned> > temp =
					intInputPrefixSet.front();
			intInputPrefixSet.pop_front();

			return temp;
	}

	std::pair<Prefix*, std::vector<std::string> > RuntimeDataManager::getNextCharInputPrefix() {
			std::pair<Prefix*, std::vector<std::string> > temp =
					charInputPrefixSet.front();
			charInputPrefixSet.pop_front();

			return temp;
	}

	bool RuntimeDataManager::isCurrentTraceUntestedForDU() {
		bool result = true;
		for (set<Trace*>::iterator ti = testedTraceList.begin(), te = testedTraceList.end(); ti != te; ti++) {
			if (currentTrace->isEqual(*ti)) {
				result = false;
				break;
			}
		}

		return result;
	}

	bool RuntimeDataManager::isMPCouldConstructed(std::string str) {
		bool couldConstructed = false;

		std::map<std::string, std::set<std::string> >::iterator it = MPMS.find(str);
		if (it != MPMS.end())
			couldConstructed = true;

		return couldConstructed;
	}

}
