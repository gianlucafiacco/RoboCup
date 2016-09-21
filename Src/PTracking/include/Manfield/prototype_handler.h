#pragma once

#include <map>
#include <stdexcept>
#include <string>

namespace manfield
{
	template<class P> class PrototypeFactory
	{
		public:
			static std::map<std::string, const P*>& getPrototypeDb();
			
			PrototypeFactory(const P* prototype) throw (std::runtime_error);
			
			static P* forName(const std::string& name) throw (std::runtime_error);
			static void freePrototypeDb();
	};
	
	#define DECLARE_PROTOTYPE(Faktory, PrototypeKlass) \
		manfield::PrototypeFactory<Faktory> factory_##PrototypeKlass(new PrototypeKlass());
	
	#define DECLARE_PROTOTYPE_SUPPORT(Faktory) \
		virtual Faktory* clone() const { return 0; }; \
		##define DEFAULT_CLONE(Klass) \
		virtual Faktory* clone() const { return new Klass(*this); }
	
	#include "prototype_handler.hpp"
}
