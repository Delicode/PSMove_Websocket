//Get number of available controllers -> psmove_count_conneted() int

//Connect available controllers or choose nr of controllers to conncet

//Default colors for controllers:
	//Controller 1: (255, 0, 0) RED
	//Controller 2: (0, 255, 0) GREEN
	//Controller 3: (0, 0, 255) BLUE
	//Controller 4: (255, 255, 0) YELLOW
	//Controller 5:	(0, 255, 255) CYAN (possible mixup with blue?)
	//Controller 6:	(255, 0, 255) PURPLE
	
//(Tracking calibration?) Other program takes care of tracking?

//Initial websocket message to server, information about connecting device
//Loop
//Poll data from the controllers
//Create Json message
//Send message using websocket


///General Header files///
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <map>
#include <cstring>
#include <sstream>
#include <fstream>
#include <new>
#include <math.h>
#include <list>

///PSMoveapi Header files///
#include "psmove.h"

///PSMoveapi tracker Header files///
#include "psmove_tracker.h"
#include "psmove_examples_opengl.h"
#include "psmove_fusion.h"

///Websocket++ Header files///
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

///Boost Header files///
#include <boost/thread/thread.hpp>

///JSON Header files///
#include "json.hpp"
using json = nlohmann::json;

///Define///
#define MAX_RETRIES 5		//Nr of time the program will try to reconnect to the websocket server before closing
#define MAX_CONTROLLERS 6	//Can possibly connect more controllers, depending on bluetooth dongle/module
#define WAIT_TIME 8			//Define wait time for limiting the nunber of messages sent per second


///Websocket++ typedef///
typedef websocketpp::client<websocketpp::config::asio_client> client;

///Websocket++ Variable///
bool disconnected = true;	//If we are disconnected from server
int retries = 0;			//If connection to server is lost we save the number of retries we do, exit program after x retries
bool start = false;			//Value that is checked before we start to send data

///Time varibles///
boost::posix_time::ptime time_start;		//When to start couting toeards the wait time
boost::posix_time::ptime time_end;			//Used to compare/determine how long time has passed
boost::posix_time::time_duration diff;		//The time difference betweene time_start and time_end

///Global variables///
char use_tracker;			//Keep track if we are tracking or not
char yes = 'y';				//For simple "yes"
char no = 'n';				//for simple "no"
float Cam_Data2[3] = {0};	//X, Y and R data from the tracker

///PSMove, initial colors///
int r[6] = {255, 0, 0, 255, 0, 255};	//Different values for different controllers, here six colors are pre-defined
int g[6] = {0, 255, 0, 255, 255, 0};	//Can be changed with the set_led_color function
int b[6] = {0, 0, 255, 0, 255, 255};	//Max nr of controllers are 5

///The PSMove controllers///
int c = psmove_count_connected();	//Get the number of connected controllers, both usb and bluetooth
PSMove **controllers = (PSMove **)calloc(c, sizeof(PSMove *));	//Allocate memory for the controllers

///Declaring functions///
void set_led_color (int controller_nr, int red, int green, int blue);	//Function declaration; to change color of led
void set_led_pwm(PSMove *move, unsigned long frequency);				//Function declaration; to dim the led
void set_rumble (PSMove *move, int intensity);							//Function declaration; to set the controller rumble on
void set_rumble_timed (PSMove *move, int intensity, int time); 			//Function declaration; that turns rumble on for a set time
void set_rumble_pattern (PSMove *move, int intensity, int pattern); 	//Function declaration; that rumbles the motor according to a pattern, ex three rumbles
void disconnect_controller (PSMove *move);								//Funciton declaration; to disconnect a controller if we want to disconnect a controller

///Classes for websocket communication///
/*
* We use Websocket++ (Websocketpp) to connect to a websocket server
* Classes are from the Websocket Utils example but have been modified to our own needs
*/

class connection_metadata {
	
public:
    typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

    connection_metadata(int id, websocketpp::connection_hdl hdl, std::string uri)
      : m_id(id)
      , m_hdl(hdl)
      , m_status("Connecting")
      , m_uri(uri)
      , m_server("N/A")
    {}

    void on_open(client * c, websocketpp::connection_hdl hdl) {
        m_status = "Open";

        client::connection_ptr con = c->get_con_from_hdl(hdl);
        m_server = con->get_response_header("Server");
    }

    void on_fail(client * c, websocketpp::connection_hdl hdl) {
        m_status = "Failed";

        client::connection_ptr con = c->get_con_from_hdl(hdl);
        m_server = con->get_response_header("Server");
        m_error_reason = con->get_ec().message();
    }

	void on_close(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Closed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "close code: " << con->get_remote_close_code() << " (" 
      << websocketpp::close::status::get_string(con->get_remote_close_code()) 
      << "), close reason: " << con->get_remote_close_reason();
    m_error_reason = s.str();
	}
	
	void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
		//std::cout << msg->get_payload().c_str() << std::endl;
        if (msg->get_opcode() == websocketpp::frame::opcode::text) {
			
			std::string incoming_msg = msg->get_payload(); 	//Convert to string
			json json_msg = json::parse(incoming_msg);		//Parse to Json
			
			int Cntrl_nr = 0;				//Initialise a variable for the controller that will be changed
			
			//Check which function is sent in the Json message
			//Extract what is needed, which device, (controller), function and variables
			//Run the function
			if (start) {
				if (json_msg.find("controller_id") != json_msg.end()) {
					auto cntr_id_nr = json_msg["controller_id"].get<std::string>();
			
					for (int j = 0; j<c; j++) {		//Loop through all connected controllers to find the number of the one we want to modify
						std::string Cntrl_ID = psmove_get_serial(controllers[j]);	//We find according to the controllers Bluetooth mac address, which is/are sent to the server in the beginning
						//const char * Chr_cntrl_id = Cntrl_ID.c_str();
				
						if(cntr_id_nr == Cntrl_ID) {				//Break when the correct controller is found
							Cntrl_nr = j;
							break;
						}
					}
				}
				if (json_msg.find("function") != json_msg.end()) {
					auto func_str = json_msg["function"].get<std::string>();
					if (func_str == "set_led_color") {				//Set the color of the LED:s , does not work if tracking controllers
						auto red = json_msg["red"].get<int>();
						auto green = json_msg["green"].get<int>();
						auto blue = json_msg["blue"].get<int>();
						//set_led_color(Cntrl_nr, red, green, blue);
						boost::thread t1(&set_led_color, Cntrl_nr, red, green, blue);
					}
					else if (func_str == "set_led_pwm") {			//Set the pwm of the LED:s , does not work if tracking controllers
						auto frequency = json_msg["frequency"].get<unsigned long>();
						//set_led_pwm(controllers[Cntrl_nr], frequency);
						boost::thread t2(&set_led_pwm, controllers[Cntrl_nr], frequency);
					}
					else if (func_str == "set_rumble") {			//Set the rumble motor on or change the intensity
						auto intensity = json_msg["intensity"].get<int>();
						//set_rumble(controllers[Cntrl_nr], intensity);
						boost::thread t3(&set_rumble, controllers[Cntrl_nr], intensity);
					}
					else if (func_str == "set_rumble_timed") {		//Set the controller to rumble a cetain duration
						auto intensity = json_msg["intensity"].get<int>();
						auto temp_time = json_msg["time"].get<int>();
						//set_rumble_timed(controllers[Cntrl_nr], intensity, temp_time);
						boost::thread t4(&set_rumble_timed, controllers[Cntrl_nr], intensity, temp_time);
					}
					else if (func_str == "set_rumble_pattern") {	//Custom rumble patterns can be made here
						int intensity = json_msg["intensity"].get<int>();
						int temp_pattern = json_msg["pattern"].get<int>();
						//set_rumble_pattern(controllers[Cntrl_nr], intensity, temp_pattern);
						boost::thread t5(&set_rumble_pattern, controllers[Cntrl_nr], intensity, temp_pattern);
					}
					else if (func_str == "disconnect_controller") {	//In case we want to disconnect a controller
						disconnect_controller(controllers[Cntrl_nr]);
					}
				}
				
			}
			if (json_msg.find("type") != json_msg.end()) {
				auto start_cmd = json_msg["type"].get<std::string>();
				if (start_cmd == "start") { //Start message from the server which will be sent to the client when the server is ready to recieve data
					start = true;
				}
			}
			incoming_msg.clear();
			json_msg.clear();
        } 
		else {
            //m_messages.push_back("<< " + websocketpp::utility::to_hex(msg->get_payload())); //If the message is not a string or similar, we only send JSON/Strings
        }
    }

    websocketpp::connection_hdl get_hdl() const {
        return m_hdl;
    }

    int get_id() const {
        return m_id;
    }

    std::string get_status() const {
        return m_status;
    }

    void record_sent_message(std::string message) {
        //m_messages.push_back(">> " + message);	//We don't need to record messages, use if you want
    }

    friend std::ostream & operator<< (std::ostream & out, connection_metadata const & data);

private:
    int m_id;
    websocketpp::connection_hdl m_hdl;
    std::string m_status;
    std::string m_uri;
    std::string m_server;
    std::string m_error_reason;
    std::vector<std::string> m_messages;
};

std::ostream & operator<< (std::ostream & out, connection_metadata const & data) {
    out << "> URI: " << data.m_uri << "\n"
        << "> Status: " << data.m_status << "\n"
        << "> Remote Server: " << (data.m_server.empty() ? "None Specified" : data.m_server) << "\n"
        << "> Error/close reason: " << (data.m_error_reason.empty() ? "N/A" : data.m_error_reason) << "\n";
    out << "> Messages Processed: (" << data.m_messages.size() << ") \n";

    std::vector<std::string>::const_iterator it;
    for (it = data.m_messages.begin(); it != data.m_messages.end(); ++it) {
        out << *it << "\n";
    }

    return out;
}

class websocket_endpoint {
	
public:
    websocket_endpoint () : m_next_id(0) {
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.clear_error_channels(websocketpp::log::elevel::all);
        m_endpoint.init_asio();
        m_endpoint.start_perpetual();
		m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);
        //m_thread.reset(new websocketpp::lib::thread(&client::run, &m_endpoint));
    }
	
	~websocket_endpoint() {
		m_endpoint.stop_perpetual();

		for (con_list::const_iterator it = m_connection_list.begin(); it != m_connection_list.end(); ++it) {
			if (it->second->get_status() != "Open") {
				// Only close open connections
				continue;
			}
			std::cout << "> Closing connection " << it->second->get_id() << std::endl;
			websocketpp::lib::error_code ec;
			m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
			if (ec) {
				std::cout << "> Error closing connection " << it->second->get_id() << ": "  
						<< ec.message() << std::endl;
			}
		}
	m_thread->join();
	}

    int connect(std::string const & uri) {
        websocketpp::lib::error_code ec;

        client::connection_ptr con = m_endpoint.get_connection(uri, ec);
        if (ec) {
            std::cout << "> Connect initialization error: " << ec.message() << std::endl;
            return -1;
        }
        int new_id = m_next_id++;
        connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(new_id, con->get_handle(), uri);
        m_connection_list[new_id] = metadata_ptr;

        con->set_open_handler(websocketpp::lib::bind(
            &connection_metadata::on_open,
            metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
        ));
		
        con->set_fail_handler(websocketpp::lib::bind(
            &connection_metadata::on_fail,
            metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
        ));
        con->set_close_handler(websocketpp::lib::bind(
            &connection_metadata::on_close,
            metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
        ));
        con->set_message_handler(websocketpp::lib::bind(
            &connection_metadata::on_message,
            metadata_ptr,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2
        ));

        m_endpoint.connect(con);

        return new_id;
    }
	
	void send(int id, std::string message) {
		websocketpp::lib::error_code ec;
		
		con_list::iterator metadata_it = m_connection_list.find(id);
		
		if (metadata_it == m_connection_list.end()) {
			std::cout << "> No connection found with id " << id << std::endl;
			return;
		}

		m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);
		
		if (ec) {
			std::cout << "> Error sending message: " << ec.message() << std::endl;
			std::cout << "> Trying again after 5 seconds \n";
			boost::this_thread::sleep(boost::posix_time::seconds(5)); //If a send fails the server has most likely crashed, wait 5 seconds if this happens then try to reconnect
			retries++;	//Keep track of the number of retries
			return;
		}
		retries = 0;
	}
	
	void close(int id, websocketpp::close::status::value code, std::string reason) {
        websocketpp::lib::error_code ec;
        
        con_list::iterator metadata_it = m_connection_list.find(id);
        if (metadata_it == m_connection_list.end()) {
            std::cout << "> No connection found with id " << id << std::endl;
            return;
        }
        
        m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
        if (ec) {
            std::cout << "> Error initiating close: " << ec.message() << std::endl;
        }
    }

	connection_metadata::ptr get_metadata(int id) const {
		con_list::const_iterator metadata_it = m_connection_list.find(id);
		if (metadata_it == m_connection_list.end()) {
			return connection_metadata::ptr();
		} else {
			return metadata_it->second;
		}
	}
	
private:
    typedef std::map<int,connection_metadata::ptr> con_list;

    client m_endpoint;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

    con_list m_connection_list;
    int m_next_id;
};

///Classes for PSMove Tracker///

class Vector3D {
    public:
        Vector3D(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}

        Vector3D &
        operator+=(const Vector3D &other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        Vector3D &
        operator-=(const Vector3D &other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        Vector3D
        operator-(const Vector3D &other) const {
            return Vector3D(x - other.x, y - other.y, z - other.z);
        }

        Vector3D &
        operator*=(float s) { x*=s; y*=s; z*=s; return *this; }

        Vector3D &
        operator/=(float s) { return (*this *= 1./s); }

        float
        length() {
            return sqrtf(x*x+y*y+z*z);
        }

        void
        normalize() {
            *this /= length();
        }

        float x;
        float y;
        float z;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();
        void update();

        void init();
        void render();
        int m_move_count;

        PSMoveTracker *m_tracker;
        PSMoveFusion *m_fusion;
        GLuint m_texture;
};

Tracker::Tracker()
    : m_move_count(psmove_count_connected()),
      m_tracker(psmove_tracker_new()),
      m_fusion(psmove_fusion_new(m_tracker, 1., 1000.))
{
		psmove_tracker_set_mirror(m_tracker, PSMove_True);
		psmove_tracker_set_exposure(m_tracker, Exposure_HIGH);
    for (int i=0; i<m_move_count; i++) {
		controllers[i] = psmove_connect_by_id(i);	//Connect the controllers
		if (use_tracker == yes) {
			 while (psmove_tracker_enable(m_tracker, controllers[i]) != Tracker_CALIBRATED); //Track the controllers
		}
    }
	if (use_tracker == no) {
		psmove_fusion_free(m_fusion);
		psmove_tracker_free(m_tracker);
	}
}

Tracker::~Tracker() {
    psmove_fusion_free(m_fusion);
    psmove_tracker_free(m_tracker);
    for (int i=0; i<m_move_count; i++) {
        psmove_disconnect(controllers[i]);
    }
}

void Tracker::update() {
	if (use_tracker == yes) {
		for (int i=0; i<m_move_count; i++) {
			Vector3D pos;
			psmove_fusion_get_position(m_fusion, controllers[i], &(pos.x), &(pos.y), &(pos.z));					//Save the position of the controller for drawing on the screen 
			psmove_fusion_get_position(m_fusion, controllers[i], &Cam_Data2[0], &Cam_Data2[1], &Cam_Data2[2]);	//Save the postion of the controller for sending to server
		}
		psmove_tracker_update_image(m_tracker);
		psmove_tracker_update(m_tracker, NULL);
	}
}

void Tracker::init() {
	if (use_tracker == yes) {
		glEnable(GL_TEXTURE_2D);
		glGenTextures(1, &m_texture);
		glBindTexture(GL_TEXTURE_2D, m_texture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	}
}

void Tracker::render() {
	if (use_tracker == yes) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		PSMoveTrackerRGBImage image = psmove_tracker_get_image(m_tracker);
		glEnable(GL_TEXTURE_2D);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
            0, GL_RGB, GL_UNSIGNED_BYTE, image.data);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		/* Draw the camera image, filling the screen */
		glColor3f(1., 1., 1.);
		glBegin(GL_QUADS);
		glTexCoord2f(0., 1.);
		glVertex2f(-1., -1.);
		glTexCoord2f(1., 1.);
		glVertex2f(1., -1.);
		glTexCoord2f(1., 0.);
		glVertex2f(1., 1.);
		glTexCoord2f(0., 0.);
		glVertex2f(-1., 1.);
		glEnd();
		glDisable(GL_TEXTURE_2D);

		/* Clear the depth buffer to allow overdraw */
		glClear(GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(psmove_fusion_get_projection_matrix(m_fusion));

		for (int i=0; i<m_move_count; i++) {
			glDisable(GL_LIGHTING);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(psmove_fusion_get_modelview_matrix(m_fusion, controllers[i]));
			glColor3f(1., 0., 0.);
			drawWireCube(1.);
		}

		glEnable(GL_LIGHTING);
		glColorMaterial ( GL_FRONT_AND_BACK, GL_EMISSION ) ;
		glEnable ( GL_COLOR_MATERIAL ) ;
		glDisable(GL_LIGHTING);
	}
}

class Renderer {
    public:
        Renderer(Tracker &tracker);
        ~Renderer();

        void init();
        void render();

		SDL_Window *m_window;
		SDL_GLContext m_glContext;
		Tracker &m_tracker;
};

Renderer::Renderer(Tracker &tracker)
    : m_window(NULL),
      m_tracker(tracker) {
	
	if (use_tracker == yes) {
		SDL_Init(SDL_INIT_VIDEO);
		if (SDL_Init(SDL_INIT_VIDEO) < 0) {
			sdlDie("Unable to initialize SDL");
		}

		m_window = SDL_CreateWindow("OpenGL Test1",
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		640, 480,
		SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
		if (m_window == NULL) {
			sdlDie("Unable to initialize SDL");
		}
		checkSDLError(__LINE__);
		m_glContext = SDL_GL_CreateContext(m_window);
		checkSDLError(__LINE__);
	}
}

Renderer::~Renderer() {
    SDL_Quit();
}

void Renderer::init() {
    glClearColor(0., 0., 0., 1.);
    glViewport(0, 0, 640, 480);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::render() {
    m_tracker.render();
	SDL_GL_SwapWindow(m_window);
}

class Main {
    public:
        Main(Tracker &tracker, Renderer &renderer);
        int exec();
    private:
        Tracker &m_tracker;
        Renderer &m_renderer;
};

Main::Main(Tracker &tracker, Renderer &renderer)
    : m_tracker(tracker),
      m_renderer(renderer)
{
}

int Main::exec() {
	int j;

	///Websocket++ Setup///
	std::string ip_address;
	int id = -1;
	websocket_endpoint endpoint;
	
	while(id < 0) {
		std::cout << "Enter IP address and port, example: ws://192.168.1.1:7651 \n";
		std::cin >> ip_address;
		id = endpoint.connect(ip_address); 
	}
	boost::this_thread::sleep(boost::posix_time::seconds(2));	//Wait for connection

	///PSMOVEAPI Setup///
	
	if (use_tracker == yes) {									//IF we want to track the controllers
		m_tracker.init();										//Initialize tracker
		m_renderer.init();										//Initialize frame renderer
	}
	
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) { 				//Checking verison
	   fprintf(stderr, "PS Move API init failed (wrong version?) \n");
       return 0;
	}
	std::string Cntrl_ID[c];									//Two strings to save the MAC adrsses in
	std::string Cntrl_ID2[c];									//This one will be modified
   
	///Initialize Controllers///
   
	for (j=0; j<c; j++) {										//Connecting all the controllers
		assert(controllers[j] != NULL);							//Check that the controller actually connected
	
	if (psmove_connection_type(controllers[j]) == Conn_USB) {	//If controller is connected with USB it cannot be polled
	    printf("> Controller %d is connected with USB, cannot poll data\n", j);
	}

	psmove_set_rate_limiting(controllers[j], PSMove_True);		//Rate limit the controller, should be on by default but will enable just in case
	
	Cntrl_ID[j] = psmove_get_serial(controllers[j]);			//Get the controllers bluetooth mac address
	Cntrl_ID2[j] = psmove_get_serial(controllers[j]);			//Get another copy of the addresses, this array will be modified for the initial JSON message
	
	psmove_set_leds(controllers[j], r[j], g[j], b[j]);			//Assign color to the controller LED
	psmove_update_leds(controllers[j]);							//Update/turn on the controller LED
	}
	
	if (controllers == NULL) {									//In case no controller successfully connected
        printf("> Could not connect to default Move controller.\n"
               "> Please connect one via Bluetooth.\n");
        exit(0);
    }
	
	///Initial contact with websocket server (NI MATE)///
	
	system("bash save_IP.sh");									//Run a bash file to get the current devices IP address, might be changed for something else
	
	std::string ip_addr;
	std::ifstream F_out;
	F_out.open("IPaddress.txt");								//Open the file that contains the IP address
	std::getline(F_out, ip_addr);								//Get the IP address
	F_out.close();
	
	ip_addr.erase(ip_addr.end()-1, ip_addr.end());				//Remove the newLine/last character from the IP address
	
	/*
	Initial message in a better from, work in progress
	j_complete["type"] = "device";
	j_complete["value"]["device_id"] = "psmove";
	j_complete["value"]["deivce_type"] = "psmove";
	j_complete["value"]["name"] = "psmove";
	j_complete["values"] = json::array({"name", "controller_id"});
	j_complete["values"]["type"] = "array";
	j_complete["values"]["data_type"] = "string";
	j_complete["values"]["count"] = c;
	j_complete["values"]["id"] = Cntrl_ID;
	j_complete["values"]["id"] = Cntrl_ID;
	
	*/
	
	///Initial JSON message template that will be sent to the Server///
	char Json[] = R"({
		"type": "device",
		"value": {
			"device_id": "psmove",
			"device_type": "psmove",
			"name": "psmove",
			"values": [ {
				"name": "controller id",
				"type": "array",
				"datatype": "string",
				"count": ,
				"ID": []
			},
			{
				"name": "accelerometer",
				"type": "vec",
				"datatype": "float",
				"vec_dimension": 3,
				"count": 1,
				"min": -32000,
				"max": 32000,
				"flags": "per_user"
			},
			{
				"name": "gyroscope",
				"type": "vec",
				"datatype": "float",
				"vec_dimension": 3,
				"count": 1,
				"min": -32000,
				"max": 32000,
				"flags": "per_user"
			},
			{
				"name": "magnetometer",
				"type": "vec",
				"datatype": "float",
				"vec_dimension": 3,
				"count": 1,
				"min": -32000,
				"max": 32000,
				"flags": "per_user"
			},
			{
				"name": "tracker",
				"type": "vec",
				"datatype": "float",
				"vec_dimension": 3,
				"count": 1,
				"min": -15,
				"max": 15,
				"flags": "per_user"
			},
			{
				"name": "buttons",
				"type": "vec",
				"datatype": "bool",
				"count": 9,
				"flags": "per_user"
			},
			{
				"name": "trigger",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			}
			],
			"Functions": {
			"set_led_color":["controller id", "red", "green", "blue"],
			"set_led_pwm":["controller id", "frequency"],
			"set_rumble":["controller id", "intensity"],
			"disconnect_controller":["controller id"]
			}
		}
		
	})";
	std::string init_msg(Json);					//Create string from the above char message
	
	///To add the controllers bluetooth address to the initial message we need to modify the strings///
	for (int i = 0; i< c; i++) {
		char controllers[1024];
		if (i == 0) {
			Cntrl_ID[i].insert(0,"\"");
			Cntrl_ID[i].append("\"");
		}
		else {
			Cntrl_ID[i].insert(0,"\"");
			Cntrl_ID[i].append("\"");
			Cntrl_ID[i].insert(0, ",");
		}
		strcpy(controllers, Cntrl_ID[i].c_str());
		init_msg.insert(229, controllers);		//Insert the controller ID:s int the initial message
	}
	
	const char *cnt = (std::to_string(c)).c_str();
	init_msg.insert(216, cnt);					//Insert the number of controllers into the initial message
	//init_msg.insert(94, ip_addr);				//Insert the IP address of the device into the initial message
	
	if (use_tracker == no) {					//Remove tracker from initial message
		init_msg.erase (785,173);
	}
	
	//std::cout << init_msg << std::endl;		//In case of bug
	
	json j_complete = json::parse(init_msg);
	std::string start_message;
	start_message = j_complete.dump();

	endpoint.send(id, start_message);			//Send the initial message	

	///Main loop, polls data and sends it to websocket server///
	
	while(!start) {								//Wait for server to send start command
		boost::this_thread::sleep(boost::posix_time::seconds(1));	
	}
	
	while (retries < MAX_RETRIES ) {			//Close after 7 retries or by pressing 'q'  || ((cvWaitKey(1) & 0xFF) != 27)

		if (use_tracker == yes ) {				//If we use the tracker we need to update the tracker and render the screen
			m_tracker.update();
			m_renderer.render();
		}

	int *Acc_Data;						//Array for Accelerometer data
	int *Gyro_Data;						//Array for the Gyroscope data
	int *Mag_Data;						//Array for the Magnetometer data
	float *Cam_Data;					//Array for the Cam Tracker data
	unsigned int *Button_Data;			//Variable for the button value
	char *array;						//Array of buttons
	unsigned char *trigger;				//Analog trigger value

	Acc_Data = new int[3];
	Gyro_Data = new int[3];
	Mag_Data = new int[3];
	Cam_Data = new float[3];
	Button_Data = new unsigned int[c];
	trigger = new unsigned char[c];
	
	time_start = boost::posix_time::microsec_clock::local_time();
	time_end = boost::posix_time::microsec_clock::local_time();
	diff = time_end - time_start;
	
	while (diff.total_milliseconds() < WAIT_TIME ) {			//Limit the data sending rate
		usleep(1);
		time_end = boost::posix_time::microsec_clock::local_time();
		diff = time_end - time_start;
	}

	for (j=0; j<c; j++) {

		array = new char[9];
		json json_j;
		std::string message;
		std::string bin_button_str;

		psmove_set_leds(controllers[j], r[j], g[j], b[j]);	//Refresh the LED color
		psmove_update_leds(controllers[j]);					//Refresh the LED state

	    if (psmove_connection_type(controllers[j]) != Conn_USB) {	//Only Bluetooth controllers can be polled
		
		while (psmove_poll(controllers[j])); //Poll the controller

		psmove_get_accelerometer(controllers[j], &Acc_Data[0], &Acc_Data[1], &Acc_Data[2]);	//Get accelerometer data
		//printf("Controller %d: accel: %5d %5d %5d\n", j, Acc_Data[0], Acc_Data[1], Acc_Data[2]);	//Print accelerometer data
		
		psmove_get_gyroscope(controllers[j], &Gyro_Data[0], &Gyro_Data[1], &Gyro_Data[2]);	//Get gyro data
		//printf("Controller %d: gyro: %5d %5d %5d\n", j, Gyro_Data[0], Gyro_Data[1], Gyro_Data[2]);	//Print gyro data

		psmove_get_magnetometer(controllers[j], &Mag_Data[0], &Mag_Data[1], &Mag_Data[2]);	//Get magnetometer data
		//printf("Controller %d: magnetometer: %5d %5d %5d\n", j, Mag_Data[0], Mag_Data[1], Mag_Data[2]);	//Print magnetometer data

		Button_Data[j] = psmove_get_buttons(controllers[j]);		//Get button data
		//printf("Controller %d: buttons: %x\n", j, Button_Data[j]);	//Print button data
		
		trigger[j] = psmove_get_trigger(controllers[j]);
		//printf("Controller %d: trigger: %x\n", j, trigger[j]);	//Print button data
		
		}
		
		//std::cout << Button_Data[0] << std::endl;
		std::bitset<22> bin_button = (Button_Data[j]);
		bin_button_str = bin_button.to_string();
		
		array[0] = bin_button_str[17];	//Triangle
		array[1] = bin_button_str[16];	//Circle
		array[2] = bin_button_str[15];	//Cross
		array[3] = bin_button_str[14];	//Square
		array[4] = bin_button_str[13];	//Start
		array[5] = bin_button_str[10];	//Select
		array[6] = bin_button_str[5];	//PS
		array[7] = bin_button_str[2];	//Move
		array[8] = bin_button_str[1];	//Trigger
		
		json_j["type"] = "data";
		json_j["value"]["device_id"] = "psmove";
		json_j["value"]["timestamp_ms"] = (time(0)*1000);
		json_j["value"]["user_1"]["accelerometer"] = {Acc_Data[0], Acc_Data[1], Acc_Data[2]};
		json_j["value"]["user_1"]["gyroscope"] = {Gyro_Data[0], Gyro_Data[1], Gyro_Data[2]};
		json_j["value"]["user_1"]["magnetometer"] = {Mag_Data[0], Mag_Data[1], Mag_Data[2]};
		if (use_tracker == yes) {
			json_j["value"]["user_1"]["tracker"] = {Cam_Data2[0], Cam_Data2[1], Cam_Data2[2]};
		}
		json_j["value"]["user_1"]["buttons"] = {array[8] - '0', array[7] - '0', array[6] - '0', array[5] - '0', array[4] - '0', array[3] - '0'
		, array[2] - '0', array[1] - '0', array[0] - '0'};
		json_j["value"]["user_1"]["trigger"] = (trigger[j]);
		
		if (retries > 0) {
			int close_code = websocketpp::close::status::service_restart;	//Reason why we are closing connection
			std::string reason = "Trying to re-connect";
			endpoint.close(id, close_code, reason);							//This will fail if server was closed, might be useless
			id = endpoint.connect(ip_address);								//Try to reconnect to the server, id will be old id + 1
			boost::this_thread::sleep(boost::posix_time::seconds(2)); 		//Have to wait a couple of seconds to reconnect
		}
		/*
		if ((array[0] - '0') == 1) {
			boost::thread t(&set_rumble_pattern, controllers[j], 255, 3);
			//t.join();
		}
		*/
		if ((array[6] - '0') == 1) {	//When PS button is pressed we stop the program by stopping the while loop
			std::cout << "PS button pressed, closing program" << std::endl;
			retries = MAX_RETRIES + 1;
			exit(0);
		}
		
		//std::cout << "here2" << std::endl;
		message = json_j.dump();
		endpoint.send(id, message);
		//std::cout << message << std::endl;
		///Cleanup before next iteration, avoid memory leaks///
		
		message.clear();
		json_j.clear();
		bin_button.reset();
		bin_button_str.clear();
		array = {0};
		delete[] array;
	}
	///Delete after each loop, avoid memory leaks///
	delete[] Acc_Data;
	delete[] Gyro_Data;
	delete[] Mag_Data;
	delete[] Cam_Data;
	delete[] Button_Data;
	delete[] trigger;
  }

	//PSMOVE Cleanup//
	
    for (j=0; j<c; j++) {	//Disconnect all the controllers when done
		set_rumble(controllers[j], 0);
        psmove_disconnect(controllers[j]);	
    }
    free(controllers);		//Free the allocated memory used by the controllers
    
	//Websocket Cleanup//
	
	int close_code = websocketpp::close::status::normal;
    std::string reason = "Quit program";

    endpoint.close(id, close_code, reason);	//Close the connection with the websocket server
	std::cout << "> Program has closed \n";
	
	return 0;    
}

///PSMove controller functions///

void set_led_color (int controller_nr, int red, int green, int blue) {	//Function to change color of led
	if (use_tracker == yes) {
		std::cout << "Tracker in use, cannot set leds" << std::endl;
	} else {
		r[controller_nr] = red;
		g[controller_nr] = green;
		b[controller_nr] = blue;
		psmove_update_leds(controllers[controller_nr]);			//Update/turn on the controller LED
	}
	return;
}

void set_led_pwm(PSMove *move, unsigned long frequency) {		//Function to dim the led
	if (use_tracker == yes) {
		std::cout << "Tracker in use, cannot set pwm" << std::endl;
	} else {
		if (frequency < 733) {									//Minimum PWM frequency is 733 Hz
			frequency = 733;
		}
		if (frequency > 24000000) {								//Maximum PWM frequency is 24 MHz
			frequency = 24000000;
		}
		psmove_set_led_pwm_frequency(move , frequency);	//Set the led PWM
	}
	return;
}

void set_rumble (PSMove *move, int intensity) {					//Function to set the controller rumble on
	if (intensity > 255) {										//Maximum rumble intensity is 255
		intensity = 255;
	}
	if (intensity < 0) {										//Minimum or off is 0
		intensity = 0;
	}
	psmove_set_rumble(move , intensity);						//Set the controller rumble
	return;
}

void set_rumble_timed (PSMove *move, int intensity, int time) {	//Funtion that turns rumble on for a set time
	if (intensity > 255) {										//Maximum rumble intensity is 255
		intensity = 255;
	}
	if (intensity < 0) {										//Minimum or off is 0
		intensity = 0;
	}
	psmove_set_rumble(move , intensity);						//Set the controller rumble 
	boost::this_thread::sleep(boost::posix_time::seconds(time));	//Let controller rumble for time, will be run in different thread
	psmove_set_rumble(move , 0);								//Turn rumble off
	return;
}

void set_rumble_pattern (PSMove *move, int intensity, int pattern) {	//Function that rumbles the motor according to a pattern, ex three rumbles
	if (pattern == 1) {
		psmove_set_rumble(move , intensity);					//Set the controller rumble 
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));	//Let controller rumble for time
		psmove_set_rumble(move , 0);							//Turn rumble off
		return;
	}
	if (pattern == 3) {
		for (int y = 0; y < 3; y++) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			psmove_set_rumble(move , intensity);				//Set the controller rumble 
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));	//Let controller rumble for time
			psmove_set_rumble(move , 0);						//Turn rumble off
		}
		return;
	}
}
void disconnect_controller (PSMove *move) {						//Funciton to disconnect a controller if we want to disconnect a controller
	c = c - 1;
	psmove_disconnect(move);
}


int main(int argc, char *argv[]) {
	std::cout << "> Nr. of controllers found: " << c << std::endl;
	
   if (c == 0) {	//If no controller is connected, won't be able to do anything so -> close program
	   std::cout << "> No controller(s) connected, please connect a controller" << std::endl;
       return 0;
   }
	
	while ((use_tracker != yes) && (use_tracker != no)) {		//Wait until we either get a 'y' or 'n'
		std::cout << "> Use tracker? [y/n] Choosing/changing led color will be disabled if tracking is enabled" << std::endl;
		std::cin >> use_tracker;
	}
	
	if (use_tracker == yes) {	//If we use the tracker we use different colors which cannot be changed
		r[0] = 0xFF, g[0] = 0x00, b[0] = 0xFF;
		r[1] = 0x00, g[1] = 0xFF, b[1] = 0xFF;
		r[2] = 0xFF, g[2] = 0xFF, b[2] = 0x00;
		r[3] = 0xFF, g[3] = 0x00, b[3] = 0x00;
		r[4] = 0x00, g[4] = 0x00, b[4] = 0xFF;
		r[5] = 0xFF, g[5] = 0xFF, b[5] = 0xFF;
	}
	
	//Might have to fix this, frame pops up and goes away when tracker does not want to be used
	Tracker tracker;
    srand(time(NULL));
    Renderer renderer(tracker);
    Main main(tracker, renderer);
	
	if (use_tracker == no) {
		SDL_Quit();		//Remove frame in case tracker is not used
	}
    
    return main.exec();
}