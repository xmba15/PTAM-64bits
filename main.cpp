#include <stdlib.h>
#include <iostream>
#include "Persistence/instances.h"
#include "System.h"

int main()
{
	std::cout << "  Welcome to PTAM " << std::endl;
	std::cout << "  --------------- " << std::endl;
	std::cout << "  Parallel tracking and mapping for Small AR workspaces" << std::endl;
	std::cout << "  Copyright (C) Isis Innovation Limited 2008 " << std::endl;
	std::cout << std::endl;
	std::cout << "  Parsing settings.cfg ...." << std::endl;
	Persistence::GUI.LoadFile("settings.cfg");

	Persistence::GUI.StartParserThread(); // Start parsing of the console input
	atexit(Persistence::GUI.StopParserThread);

	try
	{
		System s;
		s.Run();
	}
	catch (cv::Exception &e)
	{
		std::cout << std::endl;
		std::cout << "!! Failed to run system; got exception. " << std::endl;
		std::cout << "   Exception was: " << std::endl;
		const char* err_msg = e.what();
		std::cout << err_msg << std::endl;
	}
}
