/*****
 * InputGenListener.h implemented collecting all branches
 *
 */

#ifndef _INPUT_GEN_LISTENER_H_
#define _INPUT_GEN_LISTENER_H_

#include "../Core/Executor.h"
#include "RuntimeDataManager.h"
#include "BitcodeListener.h"
#include "klee/ExecutionState.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/IntrinsicInst.h"
#include "FilterSymbolicExpr.h"
#include "Event.h"
#include "Encode.h"

namespace klee {

class InputGenListener: public BitcodeListener {
public:
	InputGenListener(Executor *, RuntimeDataManager *);
	~InputGenListener();

	typedef enum {
		DFS,
		BFS
	}SearchType;
	void beforeRunMethodAsMain(ExecutionState &initialState);
	void beforeExecuteInstruction(ExecutionState &state, KInstruction *ki);
	void afterExecuteInstruction(ExecutionState &state, KInstruction *ki);
	void afterRunMethodAsMain(ExecutionState &state);
//	void createMutex(ExecutionState &state, Mutex* mutex);
//	void createCondition(ExecutionState &state, Condition* condition);
	void createThread(ExecutionState &state, Thread* thread);
	void executionFailed(ExecutionState &state, KInstruction *ki);

	ref<Expr> manualMakeSymbolic(ExecutionState& state,
			std::string name, unsigned size, bool isFloat);

	void inputGen(SearchType searchType);
	void makeBasicArgvConstraint(
			std::vector<ref<Expr> > &constraints);

	void getPrefixFromPath(std::vector<Event*>&, Event*);

	void negateTheBranch(Executor::BinTree *);
	std::string getBlockFullName(BranchInst *bi, bool boolCond);

private:
	Executor *executor;
	RuntimeDataManager * rdManager;
	FilterSymbolicExpr filter;
	Event* currentEvent;
	context z3_ctx;
	solver z3_solver;
	Encode* encode;


	void printSymbolicNode(Executor::BinTree *);
	ref<Expr> readExpr(ExecutionState &state, ref<Expr> address,
			Expr::Width size);

	void getSolveResult(std::vector<ref<Expr> >&, Executor::BinTree*);
	void freeMemoryBinTree(Executor::BinTree*);

	void negateBrRelatedToInput(Executor::BinTree *);

	void deleteMPFromThisExe();

};

}

#endif /* the end of the InputGenListener.h */
