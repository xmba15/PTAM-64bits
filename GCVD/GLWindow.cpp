#include "GLWindow.h"
#include <exception>

//#include <X11/Xlib.h>
//#include <X11/keysym.h>
//#include <GL/glx.h>


GLXInterface::Exceptions::GLWindow::CreationError::CreationError(std::string w)
{
    //what() = "GLWindow creation error: " + w;
}

GLXInterface::Exceptions::GLWindow::RuntimeError::RuntimeError(std::string w)
{
    //std::string str = 
    //what() = "GLWindow error: " + w;
}

//void GLXInterface::GLWindow::init(const cv::Size2i& size, int bpp, const std::string& title, const std::string& disp)
//{
//    Display* display = XOpenDisplay(disp == "" ? NULL : const_cast<char*>(disp.c_str()));
//    if (display == 0)
//	throw Exceptions::GLWindow::CreationError("Cannot open X display");
//
//    int visualAttributes[] = {
//	GLX_RGBA,
//	GLX_DOUBLEBUFFER,
//	GLX_RED_SIZE,      bpp/3,
//	GLX_GREEN_SIZE,    bpp/3,
//	GLX_BLUE_SIZE,     bpp/3,
//	GLX_DEPTH_SIZE,    8,
//	GLX_STENCIL_SIZE, 8,
//	None
//    };
//    XVisualInfo* visualInfo = glXChooseVisual(display, DefaultScreen(display),visualAttributes);
//    if(visualAttributes == 0) {
//	XCloseDisplay(display);
//	throw Exceptions::GLWindow::CreationError("glXChooseVisual failed");
//    }
//
//    Window rootWindow = RootWindow(display, visualInfo->screen);
//    XWindowAttributes windowAttributes;
//
//    XGetWindowAttributes(display, rootWindow, &windowAttributes);
//
//    XSetWindowAttributes attributes;
//    attributes.border_pixel = 0;
//    attributes.colormap = XCreateColormap(display, rootWindow, visualInfo->visual, AllocNone);
//    attributes.event_mask = KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask | ExposureMask;
//
//    Window window = XCreateWindow(display,
//				  rootWindow,
//				  0, 0, size.width, size.height,
//				  0, visualInfo->depth,
//				  InputOutput,
//				  visualInfo->visual,
//				  CWBorderPixel | CWColormap | CWEventMask,
//				  &attributes);
//    XStoreName(display, window, title.c_str());
//    XClassHint classHint;
//	char res_name[] = "cvd";
//    classHint.res_class = res_name;
//    classHint.res_name = (char *)title.c_str();
//    XSetClassHint(display, window, &classHint);
//    XMapWindow(display, window);
//    XEvent ev;
//    do {
//        XNextEvent(display,&ev);
//    } while (ev.type != MapNotify);
//
//    Atom delete_atom = XInternAtom(display, "WM_DELETE_WINDOW", True);
//    XSetWMProtocols(display, window, &delete_atom, 1);
//
//    GLXContext context = glXCreateContext(display, visualInfo, 0, True);
//    if (context == 0) {
//	XDestroyWindow(display, window);
//	XCloseDisplay(display);
//	throw Exceptions::GLWindow::CreationError("glXCreateContext failed");
//    }
//
//    if (glXMakeCurrent(display, window, context) == False) {
//	glXDestroyContext(display, context);
//	XDestroyWindow(display, window);
//	XCloseDisplay(display);
//	throw Exceptions::GLWindow::CreationError("glXMakeCurrent failed");
//    }
//    glLoadIdentity();
//    glViewport(0, 0, size.width, size.height);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glColor3f(1.0f,1.0f,1.0f);
//    glRasterPos2f(-1, 1);
//    glOrtho(-0.375, size.width-0.375, size.height-0.375, -0.375, -1 , 1); //offsets to make (0,0) the top left pixel (rather than off the display)
//    glPixelZoom(1,-1);
//
//    XColor black = {0, 0, 0, 0, 0, 0};
//    XFontStruct* fixed = XLoadQueryFont(display, "-misc-fixed-medium-r-*-*-12-*-*-*-*-*-*-1" );
//    Cursor null_cursor = XCreateGlyphCursor(display, fixed->fid, fixed->fid, ' ', ' ', &black, &black);
//    XFreeFont(display, fixed);
//
//    state = new State();
//    state->size = size;
//    state->title = title;
//    state->display = display;
//    state->window = window;
//    state->delete_atom = delete_atom;
//    state->null_cursor = null_cursor;
//    state->context = context;
//}
//
//void GLXInterface::GLWindow::init(const cv::Size2i& size, int bpp, const std::string& title, const std::string& disp)
//{
//    Display* display = XOpenDisplay(disp == "" ? NULL : const_cast<char*>(disp.c_str()));
//    if (display == 0)
//	throw Exceptions::GLWindow::CreationError("Cannot open X display");
//
//    int visualAttributes[] = {
//	GLX_RGBA,
//	GLX_DOUBLEBUFFER,
//	GLX_RED_SIZE,      bpp/3,
//	GLX_GREEN_SIZE,    bpp/3,
//	GLX_BLUE_SIZE,     bpp/3,
//	GLX_DEPTH_SIZE,    8,
//	GLX_STENCIL_SIZE, 8,
//	None
//    };
//    XVisualInfo* visualInfo = glXChooseVisual(display, DefaultScreen(display),visualAttributes);
//    if(visualAttributes == 0) {
//	XCloseDisplay(display);
//	throw Exceptions::GLWindow::CreationError("glXChooseVisual failed");
//    }
//
//    Window rootWindow = RootWindow(display, visualInfo->screen);
//    XWindowAttributes windowAttributes;
//
//    XGetWindowAttributes(display, rootWindow, &windowAttributes);
//
//    XSetWindowAttributes attributes;
//    attributes.border_pixel = 0;
//    attributes.colormap = XCreateColormap(display, rootWindow, visualInfo->visual, AllocNone);
//    attributes.event_mask = KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask | ExposureMask;
//
//    Window window = XCreateWindow(display,
//				  rootWindow,
//				  0, 0, size.width, size.height,
//				  0, visualInfo->depth,
//				  InputOutput,
//				  visualInfo->visual,
//				  CWBorderPixel | CWColormap | CWEventMask,
//				  &attributes);
//    XStoreName(display, window, title.c_str());
//    XClassHint classHint;
//	char res_name[] = "cvd";
//    classHint.res_class = res_name;
//    classHint.res_name = (char *)title.c_str();
//    XSetClassHint(display, window, &classHint);
//    XMapWindow(display, window);
//    XEvent ev;
//    do {
//        XNextEvent(display,&ev);
//    } while (ev.type != MapNotify);
//
//    Atom delete_atom = XInternAtom(display, "WM_DELETE_WINDOW", True);
//    XSetWMProtocols(display, window, &delete_atom, 1);
//
//    GLXContext context = glXCreateContext(display, visualInfo, 0, True);
//    if (context == 0) {
//	XDestroyWindow(display, window);
//	XCloseDisplay(display);
//	throw Exceptions::GLWindow::CreationError("glXCreateContext failed");
//    }
//
//    if (glXMakeCurrent(display, window, context) == False) {
//	glXDestroyContext(display, context);
//	XDestroyWindow(display, window);
//	XCloseDisplay(display);
//	throw Exceptions::GLWindow::CreationError("glXMakeCurrent failed");
//    }
//    glLoadIdentity();
//    glViewport(0, 0, size.width, size.height);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glColor3f(1.0f,1.0f,1.0f);
//    glRasterPos2f(-1, 1);
//    glOrtho(-0.375, size.width-0.375, size.height-0.375, -0.375, -1 , 1); //offsets to make (0,0) the top left pixel (rather than off the display)
//    glPixelZoom(1,-1);
//
//    XColor black = {0, 0, 0, 0, 0, 0};
//    XFontStruct* fixed = XLoadQueryFont(display, "-misc-fixed-medium-r-*-*-12-*-*-*-*-*-*-1" );
//    Cursor null_cursor = XCreateGlyphCursor(display, fixed->fid, fixed->fid, ' ', ' ', &black, &black);
//    XFreeFont(display, fixed);
//
//    state = new State();
//    state->size = size;
//    state->title = title;
//    state->display = display;
//    state->window = window;
//    state->delete_atom = delete_atom;
//    state->null_cursor = null_cursor;
//    state->context = context;
//}

static std::map<HWND, GLXInterface::GLWindow::State> windowMap;

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

static GLXInterface::GLWindow::EventHandler * currentHandler = NULL;

static bool windowClassRegistered = false;

void GLXInterface::GLWindow::init(const cv::Size2i& size, int bpp, const std::string& title)
{
	GLuint		PixelFormat;			// Holds The Results After Searching For A Match
	WNDCLASS	wc;						// Windows Class Structure
	DWORD		dwExStyle;				// Window Extended Style
	DWORD		dwStyle;				// Window Style
	RECT		WindowRect;				// Grabs Rectangle Upper Left / Lower Right Values
	WindowRect.left = (long)0;			// Set Left Value To 0
	WindowRect.right = (long)size.width;		// Set Right Value To Requested Width
	WindowRect.top = (long)0;				// Set Top Value To 0
	WindowRect.bottom = (long)size.height;		// Set Bottom Value To Requested Height

	HINSTANCE hInstance = GetModuleHandle(NULL);
	if (!windowClassRegistered) {
		wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;	// Redraw On Size, And Own DC For Window.
		wc.lpfnWndProc = (WNDPROC)WndProc;					// WndProc Handles Messages
		wc.cbClsExtra = 0;									// No Extra Window Data
		wc.cbWndExtra = 0;									// No Extra Window Data
		wc.hInstance = hInstance;							// Set The Instance
		wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);			// Load The Default Icon
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);			// Load The Arrow Pointer
		wc.hbrBackground = NULL;									// No Background Required For GL
		wc.lpszMenuName = NULL;									// We Don't Want A Menu
		wc.lpszClassName = "glwindow";								// Set The Class Name

		if (!RegisterClass(&wc))
			throw Exceptions::GLWindow::CreationError("Failed to register the Window Class.");
		windowClassRegistered = true;
	}

#if 0
	if (fullscreen)												// Attempt Fullscreen Mode?
	{
		DEVMODE dmScreenSettings;								// Device Mode
		memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));	// Makes Sure Memory's Cleared
		dmScreenSettings.dmSize = sizeof(dmScreenSettings);		// Size Of The Devmode Structure
		dmScreenSettings.dmPelsWidth = width;				// Selected Screen Width
		dmScreenSettings.dmPelsHeight = height;				// Selected Screen Height
		dmScreenSettings.dmBitsPerPel = bits;					// Selected Bits Per Pixel
		dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

		// Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
		if (ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN) != DISP_CHANGE_SUCCESSFUL)
		{
			// If The Mode Fails, Offer Two Options.  Quit Or Use Windowed Mode.
			if (MessageBox(NULL, "The Requested Fullscreen Mode Is Not Supported By\nYour Video Card. Use Windowed Mode Instead?", "NeHe GL", MB_YESNO | MB_ICONEXCLAMATION) == IDYES)
			{
				fullscreen = FALSE;		// Windowed Mode Selected.  Fullscreen = FALSE
			}
			else
			{
				// Pop Up A Message Box Letting User Know The Program Is Closing.
				MessageBox(NULL, "Program Will Now Close.", "ERROR", MB_OK | MB_ICONSTOP);
				return FALSE;									// Return FALSE
			}
		}
	}

	if (fullscreen)												// Are We Still In Fullscreen Mode?
	{
		dwExStyle = WS_EX_APPWINDOW;								// Window Extended Style
		dwStyle = WS_POPUP;										// Windows Style
		ShowCursor(FALSE);										// Hide Mouse Pointer
	}
	else
#endif
	{
		dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;			// Window Extended Style
		dwStyle = WS_OVERLAPPEDWINDOW;							// Windows Style
	}

	RECT oldRect = WindowRect;
	AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);		// Adjust Window To True Requested Size

																	// Create The Window
	HWND hWnd;
	if (!(hWnd = CreateWindowEx(dwExStyle,							// Extended Style For The Window
		"glwindow",							// Class Name
		NULL, //title.c_str(),								// Window Title
		dwStyle |							// Defined Window Style
		WS_CLIPSIBLINGS |					// Required Window Style
		WS_CLIPCHILDREN,					// Required Window Style
		0, 0,								// Window Position
		WindowRect.right - WindowRect.left,	// Calculate Window Width
		WindowRect.bottom - WindowRect.top,	// Calculate Window Height
		NULL,								// No Parent Window
		NULL,								// No Menu
		hInstance,							// Instance
		NULL)))								// Dont Pass Anything To WM_CREATE
	{
		throw Exceptions::GLWindow::CreationError("Window Creation Error.");
	}

	static	PIXELFORMATDESCRIPTOR pfd =				// pfd Tells Windows How We Want Things To Be
	{
		sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
		1,											// Version Number
		PFD_DRAW_TO_WINDOW |						// Format Must Support Window
		PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,							// Must Support Double Buffering
		PFD_TYPE_RGBA,								// Request An RGBA Format
		bpp,										// Select Our Color Depth
		0, 0, 0, 0, 0, 0,							// Color Bits Ignored
		0,											// No Alpha Buffer
		0,											// Shift Bit Ignored
		0,											// No Accumulation Buffer
		0, 0, 0, 0,									// Accumulation Bits Ignored
		32,											// 16Bit Z-Buffer (Depth Buffer)  
		0,											// No Stencil Buffer
		0,											// No Auxiliary Buffer
		PFD_MAIN_PLANE,								// Main Drawing Layer
		0,											// Reserved
		0, 0, 0										// Layer Masks Ignored
	};

	HDC hDC;
	if (!(hDC = GetDC(hWnd)))
		throw Exceptions::GLWindow::CreationError("Can't create a GL Device Context.");

	if (!(PixelFormat = ChoosePixelFormat(hDC, &pfd)))
		throw Exceptions::GLWindow::CreationError("Can't find a suitable PixelFormat.");

	if (!SetPixelFormat(hDC, PixelFormat, &pfd))
		throw Exceptions::GLWindow::CreationError("Can't Set The PixelFormat.");

	HGLRC hRC;
	if (!(hRC = wglCreateContext(hDC)))
		throw Exceptions::GLWindow::CreationError("Can't Create A GL Rendering Context.");

	if (!wglMakeCurrent(hDC, hRC))
		throw Exceptions::GLWindow::CreationError("Can't Activate The GL Rendering Context.");

	ShowWindow(hWnd, SW_SHOW);						// Show The Window
	SetForegroundWindow(hWnd);						// Slightly Higher Priority
	SetFocus(hWnd);									// Sets Keyboard Focus To The Window

	glLoadIdentity();
	glViewport(0, 0, size.width, size.height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glColor3f(1.0f, 1.0f, 1.0f);
	glOrtho(0, size.width, size.height, 0, -1, 1);
	glPixelZoom(1, -1);
	glRasterPos2f(0, 0);

	state = &windowMap[hWnd];
	state->parent = this;
	state->size = size;
	state->title = title;
	state->hRC = hRC;
	state->hDC = hDC;
	state->hWnd = hWnd;

	state->size_offset.width = (WindowRect.right - WindowRect.left) - (oldRect.right - oldRect.left);
	state->size_offset.height = (WindowRect.bottom - WindowRect.top) - (oldRect.bottom - oldRect.top);
	state->position_offset.x = WindowRect.left - oldRect.left;
	state->position_offset.y = WindowRect.top - oldRect.top;

	state->position = -state->position_offset;

}

GLXInterface::GLWindow::~GLWindow()
{
	if (state->hRC) {
		if (!wglMakeCurrent(NULL, NULL))
			throw Exceptions::GLWindow::RuntimeError("Release of DC and RC failed.");
		if (!wglDeleteContext(state->hRC))
			throw Exceptions::GLWindow::RuntimeError("Release Rendering Context failed.");
	}

	if (state->hDC && !ReleaseDC(state->hWnd, state->hDC))
		throw Exceptions::GLWindow::RuntimeError("Release Device Context failed.");

	if (state->hWnd && !DestroyWindow(state->hWnd))
		throw Exceptions::GLWindow::RuntimeError("Destroy Window failed.");

#if 0
	if (!UnregisterClass("glwindow", GetModuleHandle(NULL)))
		throw Exceptions::GLWindow::RuntimeError("Could not unregister Class.");
#endif

	windowMap.erase(state->hWnd);
	state = NULL;
}

cv::Size2i GLXInterface::GLWindow::size() const { return state->size; }

void GLXInterface::GLWindow::set_size(const cv::Size2i &s_){
    // we don't set state->size here, so that it changes through the event system
    // and we react to it there
	MoveWindow(state->hWnd, state->position.x + state->position_offset.x, state->position.y + state->position_offset.y, state->size.width + state->size_offset.width, state->size.height + state->size_offset.height, FALSE);
}

cv::Point2i GLXInterface::GLWindow::position() const { return state->position; }

void GLXInterface::GLWindow::set_position(const cv::Point2i &p_){
    
    state->position = p_;
	MoveWindow(state->hWnd, state->position.x + state->position_offset.x, state->position.y + state->position_offset.y, state->size.width + state->size_offset.width, state->size.height + state->size_offset.height, FALSE);
}

void GLXInterface::GLWindow::set_cursor_position(const cv::Point2i &where)
{
    //XWarpPointer(state->display, None, state->window, 0, 0, 0, 0, where.x, where.y);
}

cv::Point2i GLXInterface::GLWindow::cursor_position() const
{
    cv::Point2i where;
	POINT point;
	GetCursorPos(&point);
	where.x = point.x - state->position.x;
	where.y = point.y - state->position.y;
    return where;
}

void GLXInterface::GLWindow::show_cursor(bool show)
{
	if (show)
		ShowCursor(TRUE);
	else
		ShowCursor(FALSE);
}

std::string GLXInterface::GLWindow::title() const
{
    return state->title;
}

void GLXInterface::GLWindow::set_title(const std::string& title)
{
    state->title = title;
}

void GLXInterface::GLWindow::swap_buffers()
{
	SwapBuffers(state->hDC);
}

//inline int convertButton(const WPARAM state)
//{
//	int ret = 0;
//	if (state & MK_LBUTTON) ret |= GLXInterface::GLWindow::BUTTON_LEFT;
//	if (state & MK_MBUTTON) ret |= GLXInterface::GLWindow::BUTTON_MIDDLE;
//	if (state & MK_RBUTTON) ret |= GLXInterface::GLWindow::BUTTON_RIGHT;
//	if (state & MK_CONTROL) ret |= GLXInterface::GLWindow::BUTTON_MOD_CTRL;
//	if (state & MK_SHIFT)   ret |= GLXInterface::GLWindow::BUTTON_MOD_SHIFT;
//	return ret;
//}

inline int convertButtonState(const WPARAM state)
{
		int ret = 0;
		if (state & MK_LBUTTON) ret |= GLXInterface::GLWindow::BUTTON_LEFT;
		if (state & MK_MBUTTON) ret |= GLXInterface::GLWindow::BUTTON_MIDDLE;
		if (state & MK_RBUTTON) ret |= GLXInterface::GLWindow::BUTTON_RIGHT;
		if (state & MK_CONTROL) ret |= GLXInterface::GLWindow::BUTTON_MOD_CTRL;
		if (state & MK_SHIFT)   ret |= GLXInterface::GLWindow::BUTTON_MOD_SHIFT;
		return ret;
}

inline cv::Point convertPosition(LPARAM param)
{
	return cv::Point(GET_X_LPARAM(param), GET_Y_LPARAM(param));
}

void GLXInterface::GLWindow::handle_events(EventHandler& handler)
{
	MSG	msg;

	currentHandler = &handler; // for events only received in Window Procedure

	while (PeekMessage(&msg, state->hWnd, 0, 0, PM_REMOVE)) {
		//TranslateMessage(&msg);  // don't care for WM_CHAR/WM_DEADCHAR messages
		switch (msg.message) {
		case WM_LBUTTONDOWN:
			handler.on_mouse_down(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_LEFT);
			break;
		case WM_LBUTTONUP:
			handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_LEFT);
			break;
		case WM_MBUTTONDOWN:
			handler.on_mouse_down(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_MIDDLE);
			break;
		case WM_MBUTTONUP:
			handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_MIDDLE);
			break;
		case WM_RBUTTONDOWN:
			handler.on_mouse_down(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_RIGHT);
			break;
		case WM_RBUTTONUP:
			handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_RIGHT);
			break;
		case WM_MOUSEWHEEL:
			// positive forward, negative backward, FIXME check correspondence to X11 implementation
			handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(GET_KEYSTATE_WPARAM(msg.wParam)), (GET_WHEEL_DELTA_WPARAM(msg.wParam) > 0) ? GLWindow::BUTTON_WHEEL_UP : GLWindow::BUTTON_WHEEL_DOWN);
			break;
		case WM_MOUSEMOVE:
			handler.on_mouse_move(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam));
			break;
		case WM_KEYDOWN:
		{
			unsigned char state[256];
			GetKeyboardState(state);
			char buffer[4];
			int res = ToAscii((UINT)msg.wParam, (UINT)(msg.lParam & 0xffffff) >> 16, state, (LPWORD)buffer, 0);
			if (res == 1) {
				handler.on_key_down(*this, buffer[0]);
			}
			else {
				handler.on_key_down(*this, (int)msg.wParam);
			}
		}
		break;
		case WM_KEYUP:
		{
			unsigned char state[256];
			GetKeyboardState(state);
			char buffer[4];
			int res = ToAscii((UINT)msg.wParam, (UINT)(msg.lParam & 0xffffff) >> 16, state, (LPWORD)buffer, 0);
			if (res == 1) {
				handler.on_key_up(*this, buffer[0]);
			}
			else {
				handler.on_key_up(*this, (int)msg.wParam);
			}
		}
		break;
		case WM_DESTROY:
			handler.on_event(*this, EVENT_CLOSE);
			break;
		case WM_PAINT:
			handler.on_event(*this, EVENT_EXPOSE);
		default:
			DispatchMessage(&msg);
		}
	}

	currentHandler = NULL;

}

class SaveEvents : public GLXInterface::GLWindow::EventHandler {
private:
    std::vector<GLXInterface::GLWindow::Event>& events;
public:
    SaveEvents(std::vector<GLXInterface::GLWindow::Event>& events_) : events(events_) {}
    
    void on_key_down(GLXInterface::GLWindow&, int key) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::KEY_DOWN;
	e.which = key;
	events.push_back(e);
    }
    
    void on_key_up(GLXInterface::GLWindow&, int key) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::KEY_UP;
	e.which = key;
	events.push_back(e);
    }

    void on_mouse_move(GLXInterface::GLWindow&, cv::Point2i where, int state) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::MOUSE_MOVE;
	e.state = state;
	e.where = where;
	events.push_back(e);
    }

    void on_mouse_down(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::MOUSE_DOWN;
	e.state = state;
	e.which = button;
	e.where = where;
	events.push_back(e);
    }

    void on_mouse_up(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::MOUSE_UP;
	e.state = state;
	e.which = button;
	e.where = where;
	events.push_back(e);
    }

    void on_resize(GLXInterface::GLWindow&, cv::Size2i size) {
	
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::RESIZE;
	e.size = size;
	events.push_back(e);
    }

    void on_event(GLXInterface::GLWindow&, int event) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::EVENT;
	e.which = event;
	events.push_back(e);
    }
};

void GLXInterface::GLWindow::get_events(std::vector<Event>& events)
{
    SaveEvents saver(events);
    handle_events(saver);
}

bool GLXInterface::GLWindow::EventSummary::should_quit() const
{
    return key_down.count(VK_ESCAPE) || events.count(GLXInterface::GLWindow::EVENT_CLOSE);
}

class MakeSummary : public GLXInterface::GLWindow::EventHandler {
private:
    GLXInterface::GLWindow::EventSummary& summary;
public:
    MakeSummary(GLXInterface::GLWindow::EventSummary& summary_) : summary(summary_) {}

    void on_key_down(GLXInterface::GLWindow&, int key) {	++summary.key_down[key]; }
    void on_key_up(GLXInterface::GLWindow&, int key) { ++summary.key_up[key]; }
    void on_mouse_move(GLXInterface::GLWindow&, cv::Point2i where, int) { summary.cursor = where; summary.cursor_moved = true; }
    void on_mouse_down(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) { summary.mouse_down[button] = std::make_pair(where,state); }
    void on_mouse_up(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) { summary.mouse_up[button] = std::make_pair(where,state); }
    void on_event(GLXInterface::GLWindow&, int event) { ++summary.events[event]; }
};

void GLXInterface::GLWindow::get_events(EventSummary& summary)
{
    summary.cursor = cursor_position();
    MakeSummary ms(summary);
    handle_events(ms);
}

bool GLXInterface::GLWindow::has_events() const
{
	MSG	msg;
	return PeekMessage(&msg, state->hWnd, 0, 0, PM_NOREMOVE) != 0;
}

void GLXInterface::GLWindow::activate()
{
	if (!wglMakeCurrent(state->hDC, state->hRC))
		throw Exceptions::GLWindow::RuntimeError("wglMakeCurrent failed");
}

LRESULT CALLBACK WndProc(HWND	hWnd,			// Handle For This Window
	UINT	uMsg,			// Message For This Window
	WPARAM	wParam,			// Additional Message Information
	LPARAM	lParam)			// Additional Message Information
{
	switch (uMsg) {
	case WM_WINDOWPOSCHANGED:
		if (windowMap.count(hWnd) == 1) {
			GLXInterface::GLWindow::State & state = windowMap[hWnd];
			WINDOWPOS * pos = (WINDOWPOS *)lParam;
			cv::Size newSize(pos->cx, pos->cy);
			newSize -= state.size_offset;
			if (newSize != state.size) {
				state.size = newSize;
				state.parent->activate();
				glViewport(0, 0, state.size.width, state.size.height);
				if (currentHandler != NULL)
					currentHandler->on_resize(*state.parent, state.size);
				else
					std::cerr << "Event outside of cvd control for " << state.title << std::endl;
			}
			state.position = cv::Point(pos->x, pos->y) - state.position_offset;
			return 0;
		}
		break;
	}
	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc(hWnd, uMsg, wParam, lParam);
}