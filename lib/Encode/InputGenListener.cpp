/***
 * InputGenListener.cpp
 *
 * created on: 2016.2.28
 * 	Author: LIU Pei
 */

#include "InputGenListener.h"
#include "BitcodeListener.h"
#include "llvm/IR/Type.h"
#include "KQuery2Z3.h"
#include <iostream>
#include <z3++.h>
#include <vector>
#include <map>

using namespace llvm;

#define BinaryDebug 0
#define MakeSymbolic 0
#define BIT_WIDTH 64

namespace klee {

InputGenListener::InputGenListener(Executor *executor, RuntimeDataManager * rdManager):
		BitcodeListener(rdManager), executor(executor), rdManager(rdManager),
		   currentEvent(NULL), z3_solver(z3_ctx), encode(NULL) {
}

InputGenListener::~InputGenListener() {
}

void InputGenListener::beforeRunMethodAsMain(ExecutionState &state) {

//	endEvent = trace->path.end();
}

void InputGenListener::beforeExecuteInstruction(ExecutionState &state, KInstruction *ki) {
	Trace * trace = rdManager->getCurrentTrace();
	currentEvent = trace->path.back();
	if (currentEvent) {
		Instruction * inst = ki->inst;
		Thread * thread = state.currentThread;
		switch (inst->getOpcode()) {
		case Instruction::Br: {
#if BinaryDebug
			std::cerr << "begin Br.\n";
#endif
			//get rid of br instruction from while and for.
			BranchInst * bi = dyn_cast<BranchInst>(inst);
			if (bi->isUnconditional()) {
				break;
			}
			ref<Expr> constraint;
			ref<Expr> value = executor->eval(ki, 0, state).value;
			Expr::Width width = value->getWidth();
//			std::cerr << "width = " << width << std::endl;
			ref<Expr> concreteValue;
//			if (value->getKind() != Expr::Constant) {
//				if ((*currentEvent)->condition == true) {
//					concreteValue = ConstantExpr::create(true, width);
//				} else {
//					concreteValue = ConstantExpr::create(false, width);
//				}
//				constraint = EqExpr::create(value, concreteValue);
//				executor->evalAgainst(ki, 0, thread, concreteValue);
//			} else {
//				break;
//			}
			if (value->getKind() == Expr::Constant)
				break;
			if (currentEvent->brCondition == true) {
				concreteValue = ConstantExpr::create(true, width);
			} else {
				concreteValue = ConstantExpr::create(false, width);
			}
			constraint = EqExpr::create(value, concreteValue);
			executor->ineval(ki, 0, state, concreteValue);

			std::string bbName = inst->getParent()->getName().str();
			if (bbName[0] == 'w') {
				if (strcmp(bbName.c_str(), "while.cond") == 0 ||
						strcmp(bbName.c_str(), "while.body") == 0) {
					break;
				}
			}
			if (bbName[0] == 'f') {
				if (strcmp(bbName.c_str(), "for.cond") == 0 ||
						strcmp(bbName.c_str(), "for.body") == 0 ||
						strcmp(bbName.c_str(), "for.inc")) {
					break;
				}
			}
			if (!bi->isUnconditional()) {
				//in order to execute successfully, replace the
				//symbolic expr with the first pass concrete value.
				//for execute to the right branch.
//					ref<Expr> constraint = EqExpr::create(value, concreteValue);
					Executor::BinTree * brNode = new Executor::binTree();

					brNode->vecExpr.push_back(constraint);
					brNode->size = width;
					brNode->brTrue = currentEvent->brCondition;
					brNode->currEvent = currentEvent;
					if (executor->headSentinel == NULL) {
						executor->headSentinel = brNode;
						executor->currTreeNode = brNode;

					} else {
						Executor::BinTree * tmp = executor->currTreeNode;
						tmp->next = brNode;
						executor->currTreeNode = brNode;

					}

			}
#if BinaryDebug
			std::cerr << "end of Br\n";
#endif
			break;
		}
		case Instruction::Switch: {
#if BinaryDebug
			std::cerr << "begin Switch\n";
#endif
			//the same handling as branch statement.
			//replace symbolic value with concrete value.
			SwitchInst * si = cast<SwitchInst>(inst);
			ref<Expr> cond = executor->eval(ki, 0, state).value;
			cond->dump();
			Expr::Width width = cond->getWidth();
			if (cond->getKind() != Expr::Constant) {
				Executor::BinTree * switchStat = new Executor::binTree();
//				ref<Expr> concreteValue = (*currentEvent)->value.at(0);
				ref<Expr> concreteValue = executor->evalCurrent(ki, 0, state).value;
				ref<Expr> mainCons = EqExpr::create(cond, concreteValue);
				switchStat->isSwitch = true;
				switchStat->switchValue = concreteValue;
				switchStat->currEvent = currentEvent;
				switchStat->size = width;
				for (SwitchInst::CaseIt it = si->case_begin(), ie = si->case_end();
						it != ie; it++) {
					ref<Expr> value = executor->evalConstant(it.getCaseValue());
					if (value.compare(concreteValue) != 0) {
						ref<Expr> constraint = EqExpr::create(cond, value);
						switchStat->vecExpr.push_back(constraint);
					}

				}
				//insert the symbolic expression into the lingked list.
				if (executor->headSentinel == NULL) {
					executor->headSentinel = switchStat;
					executor->currTreeNode = switchStat;
				} else {
					Executor::BinTree * tmp = executor->currTreeNode;
					tmp->next = switchStat;
					executor->currTreeNode = switchStat;
				}
				mainCons->dump();
				executor->ineval(ki, 0, state, concreteValue);
			}
#if BinaryDenbug
			std::cerr << "end of switch statement\n";
#endif
			break;
		}
		case Instruction::Call: {
			//in order to prevent down from symbolic arguments
			//in function printf, puts and so on.
			if (!currentEvent->isFunctionWithSourceCode) {
				CallSite cs(inst);
				Function* f = currentEvent->calledFunction;
				if (f->getName() == "printf" || f->getName() == "puts") {
					unsigned numArgs = cs.arg_size();
					for (unsigned i = 1; i <= numArgs; i++) {
						ref<Expr> value = executor->eval(ki, i, state).value;
						if (value->getKind() != Expr::Constant) {
//							ref<Expr> concreteValue = (*currentEvent)->value[i - 1];
							ref<Expr> concreteValue = executor->evalCurrent(ki, i - 1, state).value;
//							std::cerr << "concrete value : " << concreteValue << std::endl;
							executor->ineval(ki, i, state, concreteValue);
						}
					}
				}
			}
			break;
		}
		}
	}
}

void InputGenListener::afterExecuteInstruction(ExecutionState &state, KInstruction *ki) {
	//handle call instruction of functions atoi, strlen and so on.
	if (currentEvent) {
		Instruction* inst = ki->inst;
//		inst->dump();
		Thread* thread = state.currentThread;
		switch (inst->getOpcode()) {
		case Instruction::Call: {
			if (!currentEvent->isFunctionWithSourceCode) {
				CallSite cs(inst);
				Function* f = currentEvent->calledFunction;
				/*
				if (f->getName() == "atoi") {
					//get the read result
					ref<Expr> address = executor->eval(ki, 1, thread).value;
					ObjectPair op;
					bool success = executor->getMemoryObject(op, state, address);
					if (success) {
						const ObjectState* os = op.second;
						ref<Expr> offset = op.first->getOffsetExpr(address);
						Expr::Width chWidth = 8;
						ref<Expr> value = os->read(offset, chWidth);
						if (value->getKind() != Expr::Constant) {
							//make atoi result symbolic.
							ref<Expr> retValue = executor->getDestCell(thread, ki).value;
							std::string retSymbol = "atoi";
							//get one char.
							ref<Expr> atoiExpr = manualMakeSymbolic(state, retSymbol, 8, false);
							executor->setDestCell(thread, ki, atoiExpr);
						}
					} else {
						assert(0 && "function atoi has problems in get ObjectState.\n");
					}
				} else if (f->getName() == "strlen") {
					ref<Expr> concreteValue = (*currentEvent)->value[1];
					executor->setDestCell(thread, ki, concreteValue);
					// we can implement to enumerate the length of the string.
					if (concreteValue->getKind() != Expr::Constant) {
						assert(0 && "strlen return value is not a constant.\n");
					}

					if (ConstantExpr* ce = dyn_cast<ConstantExpr>(concreteValue)) {
						unsigned value = ce->getZExtValue();
						std::cerr << "the value of strlen is " << value << std::endl;
					}

				} else */
				if (f->getName() == "make_input"){
					unsigned numArgs = inst->getNumOperands();
					for (unsigned i = 0; i < (numArgs - 1); i++) {
						std::string varName = inst->getOperand(i)->getName().str();
						ref<Expr> address = executor->eval(ki, i + 1, state).value;
						Expr::Width width = executor->getWidthForLLVMType(inst->getOperand(i)->getType());

						//8 bits. the problems in here. 坑
						ref<Expr> retSym = manualMakeSymbolic(state, varName, sizeof(int) * 8, false);
						ObjectPair op;
						bool success = executor->getMemoryObject(op,
								state, state.currentStack->addressSpace, address);
						if (success) {
							const ObjectState *os = op.second;
							const MemoryObject *mo = op.first;
							ObjectState *wos = state.addressSpace.getWriteable(mo, os);
							ref<Expr> offset = mo->getOffsetExpr(address);
							wos->write(offset, retSym);
						} else {
							assert(0 && "can not get the corresponding op in InputGenListener.\n");
						}
						executor->intArgvConstraints.insert(retSym);
						rdManager->intArgv.insert(make_pair(varName, -1));

					}
				}
			}
			break;
		}
		}
	}
	#if MakeSymbolic
	if (*currentEvent) {
		Instruction * inst = ki->inst;
		Thread * thread = state.currentThread;
		switch (inst->getOpcode()) {
		//handle load instruction and make symbolic
		case Instruction::Load: {
#if BinaryDebug
			std::cerr << "begin Load.\n";
#endif
			//symbolic value just like arg[number][number]
			ref<Expr> address = executor->eval(ki, 0, thread).value;
			if (ConstantExpr * ce = dyn_cast<ConstantExpr>(address)) {
				unsigned long long realAddress = ce->getZExtValue();
				int cnt = 0;
				std::map<unsigned long long, unsigned long long>::iterator it =
						executor->addressAndSize.begin(), ie = executor->addressAndSize.end();
				for (; it != ie; it++, cnt++) {
					if (realAddress >= it->first && realAddress <= it->second) {
						break;
					}
				}
				if (it != ie) {
					//make the address value symbolic
					ref<Expr> value = executor->getDestCell(thread, ki).value;
					if (ConstantExpr * vce = dyn_cast<ConstantExpr>(value)) {
						//make this value symbolic.
						//don't make '\0' symbolic.
						if (realAddress != it->second) {
							int pos = (realAddress - it->first) / (sizeof(char));
							std::stringstream ss;
							ss << "argv[" << cnt << "][" << pos  << "]";
							Expr::Width size = executor->getWidthForLLVMType(ki->inst->getType());
							ref<Expr> symbolic = manualMakeSymbolic(state, ss.str(), size, false);
							executor->setDestCell(thread, ki, symbolic);
							ss.clear();
						} else {
							//the case is the last char of the c-char-string '\0';
							//don't make this symbolic.
						}
					} else {
						//the value already symbolic
					}
				}
			} else {
				assert(0 && "the read address must be a ConstantExpr.\n");
			}
#if BinaryDebug
			std::cerr << "end Load.\n";
#endif
			break;
		}
		}
	}
#endif
//	if (currentEvent != endEvent)
//		currentEvent++;
}

void InputGenListener::afterRunMethodAsMain(ExecutionState& state) {
	std::cerr << "input generate calling start.\n";
//	encode = new Encode(rdManager);
	deleteMPFromThisExe();
	if (executor->execStatus == Executor::SUCCESS &&
			executor->executionNum < 300/*&&
			rdManager->isCurrentTraceUntestedForDU()*/)
		inputGen(InputGenListener::DFS);
//	if (rdManager->charInputPrefix.size() == 0) {
//		executor->setIsFinished();
//		Prefix *prefix = rdManager->getNextPrefix();
//		while (prefix) {
//			delete prefix;
//			prefix = NULL;
//			prefix = rdManager->getNextPrefix();
//		}
//	}
//	delete encode;
}

void InputGenListener::printSymbolicNode(Executor::BinTree * node) {
	if (node == NULL)
		return ;
	std::vector<ref<Expr> >::iterator it =
			node->vecExpr.begin(), ie = node->vecExpr.end();
	for (; it != ie; it++) {
		(*it)->dump();
	}
	printSymbolicNode(node->next);

}

void InputGenListener::createThread(ExecutionState &state, Thread *threa) {
}

void InputGenListener::executionFailed(ExecutionState &state, KInstruction *ki) {
	rdManager->getCurrentTrace()->traceType = Trace::FAILED;
}

void InputGenListener::inputGen(SearchType type) {
	switch (type) {
	case DFS: {
		//DFS search
		std::vector<ref<Expr> > constraints;
		Executor::BinTree * head = executor->headSentinel;
//		std::cerr << "start execute in DFS, constraints size = " << constraints.size() << std::endl;
		negateBrRelatedToInput(head);
		freeMemoryBinTree(head);
		executor->headSentinel = NULL;
		executor->currTreeNode = NULL;

		break;
	}
	case BFS: {
		//BFS search
		Executor::BinTree * head = executor->headSentinel;
		break;
	}
	default: {
		break;
	}
	}
}

void InputGenListener::freeMemoryBinTree(Executor::BinTree* head) {
	if (head) {
		freeMemoryBinTree(head->next);
		delete head;
		head = NULL;
	}
}

std::string InputGenListener::getBlockFullName(BranchInst *bi, bool brCond) {
	std::string ret = "";

	std::string blockName = "";
	if (brCond)
		blockName = bi->getOperand(2)->getName().str();
	else
		blockName = bi->getOperand(1)->getName().str();

	std::string funcName = bi->getParent()->getParent()->getName().str();

	ret = funcName + "." + blockName;

	return ret;
}

void InputGenListener::negateBrRelatedToInput(Executor::BinTree* head) {
	Executor::BinTree* temp = head;

	while (temp != NULL) {
		if (temp->isSwitch) {
			// has not been handled yet.
		} else {
			Event* curr = temp->currEvent;

			assert(curr->isConditionInst);
			BranchInst* bi = dyn_cast<BranchInst>(curr->inst->inst);
//			bi->dump();
			if (temp->brTrue) {
				std::string brName = getBlockFullName(bi, false);
//				std::pair<std::multimap<std::string, std::string>::iterator,
//					std::multimap<std::string, std::string>::iterator> findRes;
//
//				findRes = rdManager->MP.equal_range(brName);
//				if (findRes.first != findRes.second) {
//					std::cerr << "brName : " << brName << std::endl;
//					negateTheBranch(temp);
//				}

				if (rdManager->isMPCouldConstructed(brName)) {
//					std::cerr << "brName : " << brName << std::endl;
					negateTheBranch(temp);
				}
			} else {
				std::string brName = getBlockFullName(bi, true);

//				std::pair<std::multimap<std::string, std::string>::iterator,
//					std::multimap<std::string, std::string>::iterator> findRes;
//
//				findRes = rdManager->MP.equal_range(brName);
//				if (findRes.first != findRes.second) {
//					std::cerr << "brName : " << brName << std::endl;
//					negateTheBranch(temp);
//				}

				if (rdManager->isMPCouldConstructed(brName)) {
//					std::cerr << "brName : " << brName << std::endl;
					negateTheBranch(temp);
				}
			}
		}
		temp = temp->next;
	}
}

void InputGenListener::negateTheBranch(Executor::BinTree *node) {
	std::vector<ref<Expr> > constraints; // each new call each new vector of constraints.
	makeBasicArgvConstraint(constraints);
	Executor::BinTree * head = executor->headSentinel;
	Executor::BinTree *temp = head;

	Expr::Width width = 1;
	ref<Expr> trueExpr = ConstantExpr::create(true, width);
	ref<Expr> falseExpr = ConstantExpr::create(false, width);
	ref<Expr> constraint = ConstantExpr::create(true, width);
	while (temp != node) {
		constraints.push_back(temp->vecExpr[0]);
		temp = temp->next;
	}
	constraints.push_back(temp->vecExpr[0]);
	// get the solved result from all these constraints.
	getSolveResult(constraints, node);
}


void InputGenListener::getSolveResult(std::vector<ref<Expr> >&
		constraints, Executor::BinTree* node) {
//	encode->buildFormulaForInput(z3_solver);
	KQuery2Z3 * kq = new KQuery2Z3(z3_ctx);
	std::vector<ref<Expr> >::iterator it =
			constraints.begin(), ie = constraints.end();
//	std::cerr << "constraints in get solve size : " << constraints.size() << std::endl;
	z3_solver.push();
	while (it != (ie - 1)) {
		z3::expr res = kq->getZ3Expr((*it));
		z3_solver.add(res);
		it++;
	}
	z3::expr resTemp = kq->getZ3Expr((*it));
	z3_solver.add(!resTemp);
//	std::cerr << z3_solver << std::endl;
	check_result result = z3_solver.check();
	if (result == z3::sat) {
		std::cerr << "satisfied the constraints in get solve result.\n";
//		std::cerr << z3_solver << std::endl;
		model m = z3_solver.get_model();
		//get every char.
		std::map<std::string, char>::iterator it =
				executor->charInfo.begin(), ie = executor->charInfo.end();
		std::stringstream sr;
		for (; it != ie; it++) {
			z3::expr charExpr = z3_ctx.bv_const(it->first.c_str(), BIT_WIDTH);
			z3::expr realExpr = z3::to_expr(z3_ctx, Z3_mk_bv2int(z3_ctx, charExpr, false));
			sr << m.eval(realExpr);
			int temp = atoi(sr.str().c_str());
			char ch = toascii(temp);
			executor->charInfo[it->first] = ch;
			sr.str("");
		}
		//compute int argvs.
		std::map<std::string, unsigned> mapOfInput;
		std::map<std::string, unsigned>::iterator iit = rdManager->intArgv.begin(),
				iie = rdManager->intArgv.end();
		for (; iit != iie; iit++) {
			z3::expr tempExpr = z3_ctx.bv_const(iit->first.c_str(), BIT_WIDTH);
			z3::expr realExpr = z3::to_expr(z3_ctx, Z3_mk_bv2int(z3_ctx, tempExpr, false));
			sr << m.eval(realExpr);
			int temp = atoi(sr.str().c_str());
//			rdManager->intArgv[iit->first] = (unsigned)temp;
			mapOfInput[iit->first] = (unsigned)temp;
			sr.str("");
		}
		//need complement in order to get the corresponding concrete value.
		std::vector<std::string> argvValue;
		std::map<std::string, char>::iterator mmit = executor->charInfo.begin(),
				mmie = executor->charInfo.end();
		if (mmit != mmie) {
			std::string tempStr = mmit->first;
			char argvStr[20] = "";
			int i = 0;
			argvStr[0] = '\0';
			argvStr[i++] = mmit->second;
			mmit++;
			for (; mmit != mmie; mmit++) {
				if (tempStr[5] == (mmit->first)[5]) {
					argvStr[i++] = mmit->second;
				} else {
					argvStr[i] = '\0';
					argvValue.push_back(std::string(argvStr));
					i = 0;
					tempStr = mmit->first;
					argvStr[i] = '\0';
					argvStr[i++] = mmit->second;
				}
			}
			argvStr[i] = '\0';
			argvValue.push_back(std::string(argvStr));
		}

		// get prefix and store them in the runtime data manager.
		// use the sequence events of execution to create the prefix
		//
		std::vector<Event*> vecEvent;
		getPrefixFromPath(vecEvent, node->currEvent);
		Prefix* prefix = NULL;
//		if (rdManager->intArgv["doprint"] == 1) {
//			prefix = new Prefix(vecEvent,
//							rdManager->getCurrentTrace()->createThreadPoint, "mapOfInputAndPreix_doprint");
//			node->currEvent->inst->inst->dump();
//		}
//		else
//			prefix = new Prefix(vecEvent,
//				rdManager->getCurrentTrace()->createThreadPoint, "mapOfInputAndPreix");
//		std::map<std::string, unsigned> tempMap;
//		tempMap.insert(rdManager->intArgv.begin(), rdManager->intArgv.end());

		rdManager->addScheduleSet(prefix);

		rdManager->addcharInputPrefixSet(prefix, argvValue);
		std::map<std::string, unsigned>::iterator intIt = mapOfInput.begin();
		for (; intIt != mapOfInput.end(); intIt++) {
			std::cerr << intIt->first << " " << intIt->second << std::endl;
		}
		rdManager->addintInputPrefixSet(prefix, mapOfInput);
//		rdManager->addintInputPrefixSet(prefix, rdManager->intArgv);
//		rdManager->charInputPrefix.insert(make_pair(prefix, argvValue));
//		rdManager->intInputPrefix.insert(make_pair(prefix, tempMap));
	} else if (result == z3::unsat) {
//		std::cerr << "unsat inputGenListener.\n";
	} else {
//		std::cerr << "unknown inputGenListener.\n";
	}
	z3_solver.pop();

	delete kq;
}

ref<Expr> InputGenListener::readExpr(ExecutionState &state, ref<Expr> address,
		Expr::Width size) {
	ObjectPair op;
	executor->getMemoryObject(op, state, state.currentStack->addressSpace, address);
	const MemoryObject *mo = op.first;
	ref<Expr> offset = mo->getOffsetExpr(address);
	const ObjectState *os = op.second;
	ref<Expr> result = os->read(offset, size);
	return result;
}

ref<Expr> InputGenListener::manualMakeSymbolic(ExecutionState& state,
		std::string name, unsigned size, bool isFloat) {

	//添加新的符号变量
	const Array *array = new Array(name, size);
	ObjectState *os = new ObjectState(size, array);
	ref<Expr> offset = ConstantExpr::create(0, BIT_WIDTH);
	ref<Expr> result = os->read(offset, size);
	if (isFloat) {
		result.get()->isFloat = true;
	}
#if DEBUGSYMBOLIC
	cerr << "Event name : " << (*currentEvent)->eventName << "\n";
	cerr << "make symboic:" << name << std::endl;
	cerr << "is float:" << isFloat << std::endl;
	std::cerr << "result : ";
	result->dump();
	std::cerr << "symbolic result : " << result << std::endl;
#endif
	return result;
}


void InputGenListener::makeBasicArgvConstraint(
		std::vector<ref<Expr> > &constraints) {
	std::set<ref<Expr> >::iterator it = executor->argvSymbolics.begin(),
			ie = executor->argvSymbolics.end();
	for (; it != ie; it++) {
		//ASCII code upper and lower bound in decimal.
		ref<Expr> lowerBound = ConstantExpr::alloc(33, 8);
		ref<Expr> upperBound = ConstantExpr::alloc(126, 8);
		ref<Expr> lhs = UleExpr::create((*it), upperBound);
		ref<Expr> rhs = UgeExpr::create((*it), lowerBound);
		constraints.push_back(lhs);
		constraints.push_back(rhs);
	}

//	std::cerr << "argv symbolic size : " << constraints.size() << std::endl;
//	std::cerr << "size int argv : " << executor->intArgvConstraints.size() << std::endl;
	std::set<ref<Expr> >::iterator sit = executor->intArgvConstraints.begin(),
			sie = executor->intArgvConstraints.end();
	for (; sit != sie; sit++) {
		ref<Expr> lowerBound = ConstantExpr::alloc(0, sizeof(int) * 8);
		ref<Expr> upperBound = ConstantExpr::alloc(255, sizeof(int) * 8);
		ref<Expr> lhs = UleExpr::create((*sit), upperBound);
		ref<Expr> rhs = UgeExpr::create((*sit), lowerBound);
		constraints.push_back(lhs);
		constraints.push_back(rhs);
	}
}


void InputGenListener::getPrefixFromPath(
		std::vector<Event*>& vecEvent, Event* branchPoint) {
	Trace* trace = rdManager->getCurrentTrace();
	std::vector<Event*>::iterator it =
			trace->path.begin(), ie = trace->path.end();
	for (; it != ie; it++) {
		if ((*it) != branchPoint) {
			vecEvent.push_back((*it));
		} else {
			break;
		}
	}
}

void InputGenListener::deleteMPFromThisExe() {
	std::vector<std::string>::iterator vecIt =
			rdManager->getCurrentTrace()->sequentialBBOnceExe.begin(),
			vecItEnd = rdManager->getCurrentTrace()->sequentialBBOnceExe.end();

//		std::cerr << "all the sequence of the bbfullName\n";
//		for (std::vector<std::string>::iterator forIt =
//				trace->sequentialBBOnceExe.begin(), forIe = trace->sequentialBBOnceExe.end();
//				forIt != forIe; forIt++) {
//			std::cerr << (*forIt) << std::endl;
//		}

	for (; vecIt != vecItEnd; vecIt++) {
		for (std::vector<std::string>::iterator secVecIt =
				vecIt + 1; secVecIt != vecItEnd; secVecIt++) {
			std::map<std::string, std::set<std::string> >::iterator it =
					rdManager->MPMS.find(*vecIt);

			if (it != rdManager->MPMS.end()) {
				std::set<std::string>::iterator innerIt = it->second.find(*secVecIt);
				if (innerIt != it->second.end()) {
					if (it->second.size() == 1) {
						rdManager->MPMS.erase(it);
					} else {
						it->second.erase(innerIt);
					}
				}
			}
			std::map<std::string, std::set<std::string> >::iterator secIt =
					rdManager->MPMS.find(*secVecIt);

			if (secIt != rdManager->MPMS.end()) {
				std::set<std::string>::iterator innerIt = secIt->second.find(*vecIt);
				if (innerIt != secIt->second.end()) {
					if (secIt->second.size() == 1) {
						rdManager->MPMS.erase(secIt);
					} else {
						secIt->second.erase(innerIt);
					}
				}
			}
		}
	}
}


}
