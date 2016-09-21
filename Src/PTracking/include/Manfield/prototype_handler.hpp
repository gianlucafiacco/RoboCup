#include <exception>

template<class P> std::map<std::string, const P*>& PrototypeFactory<P>::getPrototypeDb()
{
	static std::map<std::string, const P*> db;
	
	return db;
}

template<class P> PrototypeFactory<P>::PrototypeFactory(const P* prototype) throw (std::runtime_error)
{
	std::map<std::string, const P*>& prototypeDb = PrototypeFactory<P>::getPrototypeDb();
	std::string name = prototype->toString();
	
	typename std::map<std::string, const P*>::const_iterator it = prototypeDb.find(name);
	
	if (it !=  prototypeDb.end()) throw std::runtime_error("Prototype " + name + " already present in database.");
	
	P* clone = prototype->clone();
	
	if (!clone) throw std::runtime_error("Prototype " + name + " does not support clone().\n" + "Not added to database.");
	
	std::string name2 = clone->toString();
	
	if (name2 != name) throw std::runtime_error("Converting class '" + name + "' to string returns '" + name2 + "'. \nNot added to database.");
	
	delete clone;
	
	prototypeDb[name] = prototype;
}

template<class P> P* PrototypeFactory<P>::forName(const std::string& name) throw (std::runtime_error)
{
	std::map<std::string, const P*>& prototypeDb = PrototypeFactory<P>::getPrototypeDb();
	
	typename std::map<std::string, const P*>::iterator i = prototypeDb.find(name);
	
	if ( i == prototypeDb.end()) throw std::runtime_error("Prototype '" + name + "' is not present in database.");
	
	P* clone = i->second->clone();
	
	if (!clone) throw std::runtime_error("Could not clone prototype '" + name + "'.");
	
	return clone;
}

template<class P> void PrototypeFactory<P>::freePrototypeDb()
{
	std::map<std::string, const P*>& prototypeDb = PrototypeFactory<P>::getPrototypeDb();
	
	for (typename std::map<std::string, const P*>::iterator it = prototypeDb.begin(); it != prototypeDb.end(); ++it)
	{
		delete it->second;
	}
	
	prototypeDb.clear();
}
