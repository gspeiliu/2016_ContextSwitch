/*
 * RuntimeDataManager.h
 *
 *  Created on: Jun 10, 2014
 *      Author: ylc
 */

#ifndef RUNTIMEDATAMANAGER_H_
#define RUNTIMEDATAMANAGER_H_

#include <iostream>
#include <list>
#include <set>
#include <string>
#include <vector>

#include "Prefix.h"
#include "Trace.h"



namespace klee {

class RuntimeDataManager {

	private:
		std::vector<Trace*> traceList; // store all traces;
		Trace* currentTrace; // trace associated with current execution
		std::set<Trace*> testedTraceList; // traces which have been examined
		std::list<Prefix*> scheduleSet; // prefixes which have not been examined

	public:
		unsigned allFormulaNum;
		unsigned solvingTimes;
		unsigned allGlobal;
		unsigned brGlobal;
		unsigned satBranch;
		unsigned unSatBranchBySolve;
		unsigned unSatBranchByPreSolve;

		double runningCost;
		double solvingCost;
		double satCost;
		double unSatCost;

		double DTAMCost;
		double DTAMSerialCost;
		double DTAMParallelCost;
		double DTAMhybridCost;
		double PTSCost;

		int DUInSameThread;
		int DUInDiffThread;
		int DUFromInit;


		std::set<std::pair<int, int> > DUPair;
		std::set<std::pair<int, int> > ReadFromInit;
		std::set<std::pair<int, int> > sameThreadDUSet;
		std::set<std::pair<int, int> > diffThreadDUSet;


		std::map<std::string, unsigned> intArgv;
		std::list<std::pair<Prefix*, std::vector<std::string> > > charInputPrefixSet;
		std::list<std::pair<Prefix*, std::map<std::string, unsigned> > > intInputPrefixSet;
		std::map<std::string, std::set<std::string> > MPMS;

		char **argvOfMain;
		int argcOfMain;
		unsigned allMPSet;
		std::set<std::string> inputVarSet;

		std::vector<double> allDTAMCost;
		std::vector<double> allDTAMSerialCost;
		std::vector<double> allDTAMParallelCost;
		std::vector<double> allDTAMhybridCost;
		std::vector<double> allPTSCost;

		std::vector<unsigned> DTAMSerial;
		std::vector<unsigned> DTAMParallel;
		std::vector<unsigned> DTAMhybrid;

		std::vector<unsigned> taint;
		std::vector<unsigned> taintPTS;
		std::vector<unsigned> noTaintPTS;

		std::vector<unsigned> DTAMSerialMap;
		std::vector<unsigned> DTAMParallelMap;
		std::vector<unsigned> DTAMhybridMap;
		std::vector<unsigned> TaintAndPTSMap;

		std::set<std::string> allDTAMSerialMap;
		std::set<std::string> allDTAMParallelMap;
		std::set<std::string> allDTAMhybridMap;
		std::set<std::string> allTaintMap;

		RuntimeDataManager();
		virtual ~RuntimeDataManager();

		Trace* createNewTrace(unsigned traceId);
		Trace* getCurrentTrace();
		void addScheduleSet(Prefix* prefix);
		void printCurrentTrace(bool file);
		Prefix* getNextPrefix();
		void clearAllPrefix();
		bool isCurrentTraceUntested();
		void printAllPrefix(std::ostream &out);
		void printAllTrace(std::ostream &out);

		void addcharInputPrefixSet(Prefix*, std::vector<std::string>);
		void addintInputPrefixSet(Prefix*, std::map<std::string, unsigned>);
		std::pair<Prefix*, std::vector<std::string> > getNextCharInputPrefix();
		std::pair<Prefix*, std::map<std::string, unsigned> > getNextIntInputPrefix();

		void getExplicitDU();

		bool isCurrentTraceUntestedForDU();
		bool isMPCouldConstructed(std::string str);

};

}
#endif /* RUNTIMEDATAMANAGER_H_ */
