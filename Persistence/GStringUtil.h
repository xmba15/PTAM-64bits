#pragma once

#include <vector>
#include <string>
#include <sstream>
#include "serialize.h"

namespace Persistence {

	std::string UncommentString(std::string s);
	   
	std::vector<std::string> ChopAndUnquoteString(std::string s);

	template <class T> T* ParseAndAllocate(std::string s)
	{
		std::istringstream is(s);	
		T* n = new T(Serialize::from_stream<T>(is));
		
		return n;
	}

}
