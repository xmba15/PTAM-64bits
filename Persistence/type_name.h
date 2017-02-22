#pragma once

#include <string>
#include "default.h"

namespace Persistence
{

	template <class T> std::string type_name() 
	{
#ifdef _MSC_VER
      static std::string funcname = std::string(__FUNCSIG__);
      static std::string::size_type begin = funcname.find("type_name<")+10;
      static std::string name = funcname.substr(begin, funcname.rfind(">") - begin);
#elif defined __GNUC__
	//Output format of __PRETTY_FUNCTION__ changed at 4.5.0 to have the return 
	//type included.
	#if __GNUC__ * 100 + __GNUC_MINOR__ * 10 + __GNUC_PATCHLEVEL__ < 450
      std::string funcname = std::string(__PRETTY_FUNCTION__);
      std::string bname = funcname.substr(funcname.rfind(" = ")+3);
	  std::string name = bname.substr(0, bname.length()-1);
	#else
      std::string funcname = std::string(__PRETTY_FUNCTION__);
      std::string bname = funcname.substr(funcname.rfind("T = ")+3);
	  std::string name = bname.substr(0, bname.find(", "));
	#endif
#else
	#error no good way of getting a type name
#endif
	  return name;
	}
	
	template<> inline std::string type_name<std::string>()
	{
		return "string";
	}

	
	
	
	template <class T> std::string type_name(const T&)
	{
	  return type_name<T>(); // Maybe this thing works for the OpenCV vector and Matrix
	}

	// Just adding these two overloads and specializatioon to rturn OpenCV type names directly...
	/*template<typename P, int Sz> std::string type_name<cv::Vec<P, Sz> >(const cv::Vec<P, Sz> &v) {
	  
	  return "OpenCV vector";
	}*/
	
	
	
	


}
