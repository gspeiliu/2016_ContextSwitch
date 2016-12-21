/*
 * Prefix.h
 *
 *  Created on: 2015年2月3日
 *      Author: berserker
 */

#ifndef LIB_CORE_PREFIX_H_
#define LIB_CORE_PREFIX_H_

#include "klee/Internal/Module/KInstruction.h"
#include "Event.h"

#include <llvm/Support/raw_ostream.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace klee {

	class Prefix {
		public:
			typedef std::vector<Event*>::iterator EventIterator;
			typedef std::vector<std::pair<unsigned, unsigned> >::iterator Unique2TidIterator;

		private:
			std::vector<Event*> eventList;
			std::map<Event*, uint64_t> threadIdMap;
			EventIterator position;
			std::string name;

			std::vector<std::pair<unsigned, unsigned> > unique2Tid;
			std::map<unsigned, unsigned> unique2Crt;
			Unique2TidIterator pos;
			int ContextSwitch;

		public:
			Prefix(std::vector<Event*>& eventList,
					std::map<Event*, uint64_t>& threadIdMap, std::string name);
			Prefix(std::vector<Event*>& eventList,
					std::map<Event*, uint64_t>& threadIdMap, std::string name, int ContextSwitch);
			Prefix(std::vector<std::pair<unsigned, unsigned> >& tidList,
					std::map<unsigned, unsigned>& crtPoint, std::string name, int ContextSwitch);
			virtual ~Prefix();
			std::vector<Event*>* getEventList();
			void increasePosition();
			void reuse();
			bool isFinished();
			Unique2TidIterator begin();
			Unique2TidIterator end();
			Unique2TidIterator current();
			uint64_t getNextThreadId();
			unsigned getCurrentEventThreadId();
			void print(std::ostream &out);
			void print(llvm::raw_ostream &out);
			KInstruction* getCurrentInst();
			std::string getName();

			int getContextSwitch();
	};

} /* namespace klee */

#endif /* LIB_CORE_PREFIX_H_ */
