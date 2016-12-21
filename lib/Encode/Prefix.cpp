/*
 * Prefix.cpp
 *
 *  Created on: 2015年2月3日
 *      Author: berserker
 */

#include "Prefix.h"

#include <llvm/IR/Instruction.h>
#include <iterator>

#include "../../include/klee/Internal/Module/InstructionInfoTable.h"

using namespace ::std;
using namespace ::llvm;

namespace klee {

	Prefix::Prefix(vector<Event*>& eventList, std::map<Event*, uint64_t>& threadIdMap, std::string name) :
			eventList(eventList), threadIdMap(threadIdMap), name(name), ContextSwitch(0) {
		position = this->eventList.begin();
	}

	Prefix::Prefix(std::vector<Event*>& eventList, std::map<Event*, uint64_t>& threadIdMap, std::string name, int ContextSwitch) :
			eventList(eventList), threadIdMap(threadIdMap), name(name), ContextSwitch(ContextSwitch) {
		position = this->eventList.begin();
	}

	Prefix::Prefix(std::vector<std::pair<unsigned, unsigned> >& tidList,
			std::map<unsigned, unsigned>& crtPoint, std::string name, int ContextSwitch) :
		unique2Tid(tidList), unique2Crt(crtPoint), name(name), ContextSwitch(ContextSwitch) {
		pos = this->unique2Tid.begin();
	}
	void Prefix::reuse() {
		position = this->eventList.begin();
		pos = this->unique2Tid.begin();
	}

	Prefix::~Prefix() {

	}

	vector<Event*>* Prefix::getEventList() {
		return &eventList;
	}

	void Prefix::increasePosition() {
		if (!isFinished()) {
			position++;
			pos++;
		}
	}

	bool Prefix::isFinished() {
//		return position == eventList.end();
		return pos == unique2Tid.end();
	}

	Prefix::Unique2TidIterator Prefix::begin() {
//		return eventList.begin();
		return unique2Tid.begin();
	}

	Prefix::Unique2TidIterator Prefix::end() {
//		return eventList.end();
		return unique2Tid.end();
	}

	Prefix::Unique2TidIterator Prefix::current() {
		return Prefix::Unique2TidIterator(pos);
	}

	uint64_t Prefix::getNextThreadId() {
		assert(!isFinished());
//		Event* event = *position;
//		map<Event*, uint64_t>::iterator ti = threadIdMap.find(event);
//		return ti->second;
		return unique2Crt[pos->first];
	}

	unsigned Prefix::getCurrentEventThreadId() {
		assert(!isFinished());
//		Event* event = *position;
//		return event->threadId;
		return pos->second;
	}

	void Prefix::print(ostream &out) {
		for (vector<Event*>::iterator ei = eventList.begin(), ee = eventList.end(); ei != ee; ei++) {
			Event* event = *ei;
			out << "thread" << event->threadId << " " << event->inst->info->file << " " << event->inst->info->line << ": "
					<< event->inst->inst->getOpcodeName();
			map<Event*, uint64_t>::iterator ti = threadIdMap.find(event);
			if (ti != threadIdMap.end()) {
				out << "\n child threadId = " << ti->second;
			}
			out << endl;
		}
		out << "prefix print finished\n";
	}

	void Prefix::print(raw_ostream &out) {
		for (vector<Event*>::iterator ei = eventList.begin(), ee = eventList.end(); ei != ee; ei++) {
			Event* event = *ei;
			out << "thread" << event->threadId << " " << event->inst->info->file << " " << event->inst->info->line << ": ";
			event->inst->inst->print(out);
			map<Event*, uint64_t>::iterator ti = threadIdMap.find(event);
			if (ti != threadIdMap.end()) {
				out << "\n child threadId = " << ti->second;
			}
			out << '\n';
		}
		out << "prefix print finished\n";
	}

	KInstruction* Prefix::getCurrentInst() {
		assert(!isFinished());
		Event* event = *position;
		return event->inst;
	}
	std::string Prefix::getName() {
		return name;
	}

	int Prefix::getContextSwitch() {
		return ContextSwitch;
	}

} /* namespace klee */
