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
#include <new>

///PSMoveapi Header files///
//#include <opencv2/core/core_c.h>
//#include <opencv2/highgui/highgui_c.h>
#include "psmove.h"

///Websocket++ Header files///
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

///Boost Header files///
#include <boost/thread/thread.hpp>

///JSON Header files///
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
using namespace rapidjson;

///Websocket++ typedef///
typedef websocketpp::client<websocketpp::config::asio_client> client;

///Websocket++ functions///
bool disconnected = true;

///PSMove, initial colors///
int r[6] = {255, 0, 0, 255, 0, 255};	//Different values for different controllers
int g[6] = {0, 255, 0, 255, 255, 0};	//Can be changed with the set_led_color function
int b[6] = {0, 0, 255, 0, 255, 255};

///The PSMove controllers///
int c = psmove_count_connected();	//Get the number of connected controllers, both usb and bluetooth
PSMove **controllers = (PSMove **)calloc(c, sizeof(PSMove *));	//Allocate memory for the controllers

int retries = 0;

///Declaring functions///
void set_led_color (int controller_nr, int red, int green, int blue);
void set_led_pwm(PSMove *move, unsigned long frequency);
void set_rumble (PSMove *move, int intensity);
void disconnect_controller (PSMove *move);

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
	std::cout << msg->get_payload().c_str();
        if (msg->get_opcode() == websocketpp::frame::opcode::text) {
			Document document;
            //m_messages.push_back("<< " + msg->get_payload()); //We send too many messages, causes memory leak
			std::cout << msg->get_payload();
			
			std::string incoming_msg = msg->get_payload(); //Convert to string
			const char * json_msg = incoming_msg.c_str();  //Convert to char so that it can be parsed
			
			document.Parse(json_msg);	//Parsing into Json
			assert(document.IsObject());
			int Cntrl_nr = 0;			//Initialise a variable for the controller that will be modified
			
			for (int j = 0; j<c; j++) {	//Loop through all connected controllers to find the number of the one we want to modify
				std::string Cntrl_ID = psmove_get_serial(controllers[j]);
				const char * Chr_cntrl_id = Cntrl_ID.c_str();
				
				if(document["Controller ID"] == Chr_cntrl_id) {	//Break when the correct controller is found
					Cntrl_nr = j;
					break;
				}
			}
			//Check which function is sent in the Json message
			//Extract what is needed, which device, (controller), function and variable
			//Run the function
			if (document["Function"] == "set_led_color") {
				int red = document["Red"].GetInt();
				int green = document["Green"].GetInt();
				int blue = document["Blue"].GetInt();
				set_led_color(Cntrl_nr, red, green, blue);
			}
			else if (document["Function"] == "set_led_pwm") {
				unsigned long frequency = document["Frequency"].GetDouble();
				set_led_pwm(controllers[Cntrl_nr], frequency);
			}
			else if (document["Function"] == "set_rumble") {
				int intensity = document["Intensity"].GetInt();
				set_rumble(controllers[Cntrl_nr], intensity);
			}	
			else if (document["Function"] == "disconnect_controller") {
				disconnect_controller(controllers[Cntrl_nr]);
			}
		
        } else {
            //m_messages.push_back("<< " + websocketpp::utility::to_hex(msg->get_payload())); //If the message is not a string or similar
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
        //m_messages.push_back(">> " + message);	
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
			boost::this_thread::sleep(boost::posix_time::seconds(5));
			retries++;
			return;
		}
		retries = 0;
		//metadata_it->second->record_sent_message(message); //Save sent messages, if sending many it uses a lot of memory/ causes memory leak
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

void set_led_color (int controller_nr, int red, int green, int blue) {
	r[controller_nr] = red;
	g[controller_nr] = green;
	b[controller_nr] = blue;
	psmove_update_leds(controllers[controller_nr]);			//Update/turn on the controller LED
}

void set_led_pwm(PSMove *move, unsigned long frequency) {
	if (frequency < 733) {	//Minimum PWM frequency is 733 Hz
		frequency = 733;
	}
	if (frequency > 24000000) {	//Maximum PWM frequency is 24 MHz
		frequency = 24000000;
	}
	psmove_set_led_pwm_frequency(move , frequency);	//Set the led PWM
}

void set_rumble (PSMove *move, int intensity) {
	if (intensity > 255) {	//Maximum rumble intensity is 255
		intensity = 255;
	}
	if (intensity < 0) {	//Minimum or off is 0
		intensity = 0;
	}
	psmove_set_rumble(move , intensity);	//Set the controller rumble 
}
/*	
int change_port (websocket_endpoint *endpoint, std::string ip) {
	int id = endpoint->connect(ip);
	return id;
}
*/
void disconnect_controller (PSMove *move) {
	psmove_disconnect(move);
}


int main(int argc, char* argv[]) {
	
	//Websocket++ Setup//
	std::string ip_address;
	int id = -1;
	websocket_endpoint endpoint;
	while(id < 0) {
		std::cout << "Enter IP address and port, example: ws://192.168.1.1:7651 \n";
		std::getline(std::cin, ip_address);
		id = endpoint.connect(ip_address); 
	}
	
	//PSMOVEAPI Setup//
   
	//int c = psmove_count_connected();	//Get the number of connected controllers, both usb and bluetooth
	//PSMove **controllers = (PSMove **)calloc(c, sizeof(PSMove *));	//Allocate memory for the controllers
   
   int j;

     printf("> Nr. of controllers found: %d \n", c);
   
   if (c == 0) {	//If no controller is connected, close program
	   printf("> No controller(s) connected, please connect a controller \n");
       return 0;
   }
   
   if (!psmove_init(PSMOVE_CURRENT_VERSION)) { //Checking verison
        fprintf(stderr, "PS Move API init failed (wrong version?) \n");
        return 0;
   }
	std::string Cntrl_ID[c];
   
   printf("> Connecting %d controllers, setting color(s) \n", c);
   
   for (j=0; j<c; j++) {	//Connecting all the controllers
	
    controllers[j] = psmove_connect_by_id(j);	//Connect to the controller
	
	if (psmove_connection_type(controllers[j]) == Conn_USB) {	//If controller is connected with USB it cannot be polled
	    printf("> Controller %d is connected with USB, cannot poll data \n", j);
	}
	assert(controllers[j] != NULL);				//Check that the controller actually connected
	psmove_set_rate_limiting(controllers[j], PSMove_True);//Rate limit the controller, should be on by default but will enable just in case
	psmove_set_leds(controllers[j], r[j], g[j], b[j]);//Assign colot to the controller LED
	psmove_update_leds(controllers[j]);			//Update/turn on the controller LED
	Cntrl_ID[j] = psmove_get_serial(controllers[j]);
	}
	
	if (controllers == NULL) {	//In case no controller successfully connected
        printf("> Could not connect to default Move controller.\n"
               "> Please connect one via Bluetooth.\n");
        exit(1);
    }
	
	///Initial contact with websocket server (NI MATE)///
	/*StringBuffer Sb;
	Writer<StringBuffer> writer(Sb);
	
	writer.StartObject();
	writer.Key("Device Name");
	writer.String("PSMOVE");
	writer.Key("Device ID");
	writer.String("");
	writer.EndObject();
	*/	
	///Main loop, polls data and sends it to websocket server///
	//StringBuffer s;
	
   while (retries < 6) {	//Close after 7 retries
   
	int *Acc_Data;//[3];	//Array for Accelerometer data
	int *Gyro_Data;//[3];	//Array for the gyroscope data
	int *Mag_Data;//[3];	//Array for the Magnetometer data
	int *Button_Data;	//Variable for the button value
	
	Acc_Data = new int[3];
	Gyro_Data = new int[3];
	Mag_Data = new int[3];
	Button_Data = new int[1];
	
	
	for (j=0; j<c; j++) {
		std::string message;
		psmove_set_leds(controllers[j], r[j], g[j], b[j]);	//Refresh the LED color
        psmove_update_leds(controllers[j]);					//Refresh the LED state
		
	    if (psmove_connection_type(controllers[j]) != Conn_USB) {	//Only Bluetooth controllers can be polled
			
		psmove_poll(controllers[j]); //Poll the controller

		psmove_get_accelerometer(controllers[j], &Acc_Data[0], &Acc_Data[1], &Acc_Data[2]);	//Get accelerometer data
		//printf("Controller %d: accel: %5d %5d %5d\n", j, Acc_Data[0], Acc_Data[1], Acc_Data[2]);	//Print accelerometer data
		
		psmove_get_gyroscope(controllers[j], &Gyro_Data[0], &Gyro_Data[1], &Gyro_Data[2]);	//Get gyro data
		//printf("Controller %d: gyro: %5d %5d %5d\n", j, Gyro_Data[0], Gyro_Data[1], Gyro_Data[2]);	//Print gyro data

		psmove_get_magnetometer(controllers[j], &Mag_Data[0], &Mag_Data[1], &Mag_Data[2]);	//Get magnetometer data
		//printf("Controller %d: magnetometer: %5d %5d %5d\n", j, Mag_Data[0], Mag_Data[1], Mag_Data[2]);	//Print magnetometer data

		Button_Data[0] = psmove_get_buttons(controllers[j]);	//Get button data
		//printf("Controller %d: buttons: %x\n", j, Button_Data);	//Print button data
        
		}
		
		StringBuffer s;
		Writer<StringBuffer> writer(s);
		
		writer.StartObject();
		writer.Key("ID");
		writer.String(Cntrl_ID[j].c_str());
		writer.Key("Accelerometer");
		writer.StartArray();
			writer.Double(Acc_Data[0]);
			writer.Double(Acc_Data[1]);
			writer.Double(Acc_Data[2]);
		writer.EndArray();
		writer.Key("Gyroscope");
		writer.StartArray();
			writer.Double(Gyro_Data[0]);
			writer.Double(Gyro_Data[1]);
			writer.Double(Gyro_Data[2]);
		writer.EndArray();
		writer.Key("Magnetometer");
		writer.StartArray();
			writer.Double(Mag_Data[0]);
			writer.Double(Mag_Data[1]);
			writer.Double(Mag_Data[2]);
		writer.EndArray();
		writer.Key("Button");
		writer.Uint(Button_Data[0]);
		writer.EndObject();
		/*
		if (retries > 0) {
			int close_code = websocketpp::close::status::service_restart;
			std::string reason = "Trying to re-connect";
			endpoint.close(id, close_code, reason);
			id = endpoint.connect(ip_address);
		}
		*/
		message = s.GetString();
		endpoint.send(id, message);
		message.clear();
		
	}
	delete[] Acc_Data;
	delete[] Gyro_Data;
	delete[] Mag_Data;
	delete[] Button_Data;
   }

	//PSMOVE Cleanup//
	
    for (j=0; j<c; j++) {	//Disconnect all the controllers when done
        psmove_disconnect(controllers[j]);	
    }
	
    free(controllers);	//Free the allocated memory used by the controllers
    //Websocket Cleanup//
	
	int close_code = websocketpp::close::status::normal;
    std::string reason = "Quit program";

    endpoint.close(id, close_code, reason);
	std::cout << "> Program has closed \n";
	
   return 0;
}