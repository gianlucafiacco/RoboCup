#pragma once

#include <map>
#include <string>

namespace GMapping
{
	class AutoVal
	{
		private:
			std::string m_value;
			
		protected:
			std::string toLower(const std::string& source) const;
			
		public:
			AutoVal() {;}
			
			explicit AutoVal(const std::string&);
			explicit AutoVal(float);
			explicit AutoVal(int);
			explicit AutoVal(unsigned int);
			explicit AutoVal(bool);
			explicit AutoVal(const char*);
			
			AutoVal(const AutoVal&);
			AutoVal& operator=(const AutoVal&);
			
			AutoVal& operator= (float);
			AutoVal& operator= (int);
			AutoVal& operator= (unsigned int);
			AutoVal& operator= (bool);
			AutoVal& operator= (const std::string&);
			
			operator std::string() const;
			operator float() const;
			operator int() const;
			operator unsigned int() const;
			operator bool() const;
	};
	
	class ConfigFile
	{
		std::map<std::string,AutoVal> m_content;
		
		protected:
			void insertValue(const std::string& section,const std::string& entry,const std::string& thevalue);
			std::string toLower(const std::string& source) const;
			std::string trim(const std::string& source,char const* delims = " \t\r\n") const;
			std::string truncate(const std::string& source,const char* atChar) const;
			
		public:
			ConfigFile();
			ConfigFile(const std::string& configFile);
			ConfigFile(const char* configFile);
			
			void dumpValues(std::ostream& out);
			
			bool read(const std::string& configFile);
			bool read(const char* configFile);
			
			const AutoVal& value(const std::string& section,const std::string& entry) const;
			const AutoVal& value(const std::string& section,const std::string& entry,float def);
			const AutoVal& value(const std::string& section,const std::string& entry,const char* def);
			const AutoVal& value(const std::string& section,const std::string& entry,bool def);
			const AutoVal& value(const std::string& section,const std::string& entry,int def);
			const AutoVal& value(const std::string& section,const std::string& entry,unsigned int def);
			const AutoVal& value(const std::string& section,const std::string& entry,const std::string& def);
	};
}
