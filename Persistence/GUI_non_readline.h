#pragma once

#include <string>
#include <pthread.h>

namespace Persistence
{

	class spawn_non_readline_thread
	{
		public:
			spawn_non_readline_thread(const std::string&);
			~spawn_non_readline_thread();

		private:
			static bool running;
			static bool quit;
			static std::string quit_callback;
			pthread_t cmd;
			bool 	  none;
			static  void* proc(void*);
	};


};
