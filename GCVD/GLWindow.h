#include <string>
#include <vector>
#include <map>
#include <Windows.h>
#include <windowsx.h>
#include <GL/GL.h>

#ifdef _WIN32
#include <Windows.h>
#endif

#include <opencv2/opencv.hpp>

namespace GLXInterface {

    namespace Exceptions
    {
	/// %Exceptions specific to CVD::GLWindow
	/// @ingroup gException
	namespace GLWindow
	{
	    /// Base class for all CVD::GLWindow exceptions
	    /// @ingroup gException
	    //struct All: public CVD::Exceptions::All{
	    //};

	    struct All: public std::exception {
	    
	    };
	      
	    /// An exception occurred during initialisation
	    struct CreationError: public All
	    {
		CreationError(std::string w); ///< Construct from error string
	    };

	    /// An exception occurred during run-time
	    struct RuntimeError: public All
	    {
		RuntimeError(std::string w); ///< Construct from error string
	    };
	}
    }

    /// An object that creates a window and a GL context attached to that window, and manages its events.
    class GLWindow {
   
	public:
  
	/// Symbols for mouse buttons and modifiers
	enum MouseButton { BUTTON_LEFT=1, BUTTON_MIDDLE=2, BUTTON_RIGHT=4, BUTTON_MOD_CTRL=8, BUTTON_MOD_SHIFT=0x10, BUTTON_WHEEL_UP=0x20, BUTTON_WHEEL_DOWN=0x40 };
	/// Symbols for window events
	enum EventType { EVENT_CLOSE, EVENT_EXPOSE };

	/// Abstract base class for event handlers.  Subclass this and override to implement a handler.
	class EventHandler {
	  public:
	    virtual ~EventHandler() {}
	    /// Called for key press events
	    virtual void on_key_down(GLWindow&, int /*key*/) {}
	    /// Called for key release events
	    virtual void on_key_up(GLWindow& /*win*/, int /*key*/) {}
	    /// Called for mouse movement events
	    virtual void on_mouse_move(GLWindow& /*win*/, cv::Point /*where*/, int /*state*/) {}
	    /// Called for mouse button press events
	    virtual void on_mouse_down(GLWindow& /*win*/, cv::Point /*where*/, int /*state*/, int /*button*/) {}
	    /// Called for mouse button release events
	    virtual void on_mouse_up(GLWindow& /*win*/, cv::Point /*where*/, int /*state*/, int /*button*/) {}
	    /// Called for window resize events
	    virtual void on_resize(GLWindow& /*win*/, cv::Size /*size*/) {}
	    /// Called for general window events (such as EVENT_CLOSE)
	    virtual void on_event(GLWindow& /*win*/, int /*event*/) {}
	};

	struct Event {
	    enum Type { KEY_DOWN, KEY_UP, MOUSE_MOVE, MOUSE_DOWN, MOUSE_UP, RESIZE, EVENT };
	    Type type;
	    int which, state;
	    cv::Point where;
	    cv::Size size;
	};

	/// A summary of multiple events
	struct EventSummary {
	    
	    EventSummary() : cursor(-1,-1), cursor_moved(false) {}
	    /// key->frequency mapping for key presses and releases
	    std::map<int,int> key_down, key_up;
	    typedef std::map<int,int>::const_iterator key_iterator;
	    /// button->frequency mapping for mouse presses and releases
	    std::map<int,std::pair<cv::Point, int> > mouse_down, mouse_up;
	    typedef std::map<int,std::pair<cv::Point,int> >::const_iterator mouse_iterator;
	    /// Generic window events -> frequency
	    std::map<int,int> events;
	    /// Reset the summary
	    void clear() { *this = EventSummary(); }
	    /// Has escape been pressed or the close button pressed?
	    bool should_quit() const;
	    /// last seen cursor position from mouse_move
	    cv::Point cursor;
	    /// was the cursor moved during the recording of the history
	    bool cursor_moved;
	};

	/// Construct a GLWindow of the given size and colour depth, with the given title.
	/// A double-buffered GL context is associated with the window.
	/// @param size    Window size
	/// @param bpp     Colour depth
	/// @param title   Window title
	/// @param display X11 display string, passed to XOpenDisplay. "" Is used to indicate NULL. This is ignored for non X11 platforms. 
	GLWindow(const cv::Size& size, int bpp=24, const std::string& title="GLWindow", const std::string& display="") {
	  init(size, bpp, title);
	}
	///@overload
	GLWindow(const cv::Size& size, const std::string& title, int bpp=24, const std::string& display="") {
	  init(size, bpp, title);
	}

	~GLWindow();
	/// Get the size
	cv::Size size() const;
	/// Set the size
	void set_size(const cv::Size &);
	/// Get the position
	cv::Point position() const;
	/// Set the position
	void set_position(const cv::Point &);
	/// Set the mouse cursor position
	void set_cursor_position(const cv::Point& where);
	/// Get the mouse cursor position
	cv::Point cursor_position() const;
	/// Show (or hide) the cursor
	void show_cursor(bool show=true);
	/// Hide the cursor
	void hide_cursor() { show_cursor(false); }
	/// Get the title
	std::string title() const;
	/// Set the title
	void set_title(const std::string& title);
	/// Swap the front and back buffers
	void swap_buffers();
	/// Handle events in the event queue by calling back to the specified handler.
	void handle_events(EventHandler& handler);
	/// Store all events in the event queue into Event objects.
	void get_events(std::vector<Event>& events);
	/// Make a summary of the events in the queue.
	void get_events(EventSummary& summary);
	/// @returns true if events are pending
	bool has_events() const;
	/// Make this GL context active
	void activate();
	/// Make this GL context active
	void make_current() { activate(); }
    
    struct State {
      cv::Size size;
      cv::Point position;
      std::string title;
	  cv::Size size_offset;
	  cv::Point position_offset;

	  GLWindow * parent;

	  HGLRC   hRC;
	  HDC     hDC;
	  HWND    hWnd;
    };
	   
    private:
	State* state;
	void init(const cv::Size& sz, int bpp, const std::string& title);

    };
}
