//Get number of available controllers -> psmove_count_conneted() int

//Connect available controllers or choose nr of controllers to conncet

//Colors for controllers:
	//Controller 1: (255, 0, 0) RED
	//Controller 2: (0, 255, 0) GREEN
	//Controller 3: (0, 0, 255) BLUE
	//Controller 4: (255, 255, 0) YELLOW
	//Controller 5:	(0, 255, 255) CYAN (possible mixup with blue?)
	//Controller 6:	(255, 0, 255) PURPLE
//(Tracking calibration?) Other program takes care of tracking?

//Loop
//Poll data from the controllers

//Send data using websocket

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

///PSMoveapi Header files///
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include "psmove.h"

///Websocket++ Header files///
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

///JSON Header files///
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
using namespace rapidjson;
///Websocket++ typedef///
typedef websocketpp::client<websocketpp::config::asio_client> client;

///Websocket++ functions///

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
        if (msg->get_opcode() == websocketpp::frame::opcode::text) {
            m_messages.push_back("<< " + msg->get_payload());
        } else {
            m_messages.push_back("<< " + websocketpp::utility::to_hex(msg->get_payload()));
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
        m_messages.push_back(">> " + message);
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
			return;
		}

		metadata_it->second->record_sent_message(message);
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



///PSMove variables///

int Acc_Data[3];	//Array for Accelerometer data
int Gyro_Data[3];	//Array for the gyroscope data
int Mag_Data[3];	//Array for the Magnetometer data
int Button_Data;	//Variable for the button value


int main(int argc, char* argv[]) {
	//Websocket++//
	
	
	websocket_endpoint endpoint;
	
	int id = endpoint.connect("ws://echo.websocket.org:9002"); //Insert uri here!
	//PSMOVEAPI//
   
   int r[6] = {255, 0, 0, 255, 0, 255};	//Different values for different controllers
   int g[6] = {0, 255, 0, 255, 255, 0};	
   int b[6] = {0, 0, 255, 0, 255, 255};
   int j, c;

   c = psmove_count_connected();	//Get the number of connected controllers, both usb and bluetooth
   printf("Nr. of controllers found: %d \n", c);
   
   if (c == 0) {	//If no controller is connected, close program
	   printf("No controller(s) connected, please connect a controller");
       return 0;
   }
   
   if (!psmove_init(PSMOVE_CURRENT_VERSION)) { //Checking verison
        fprintf(stderr, "PS Move API init failed (wrong version?)\n");
        return 0;
   }

   PSMove **controllers = (PSMove **)calloc(c, sizeof(PSMove *));	//Allocate memory for the controllers

   printf("Connecting %d controllers, setting color(s) \n", c);
   
   for (j=0; j<c; j++) {	//Connecting all the controllers
   
	if (psmove_connection_type(controllers[j]) == Conn_USB) {	//If controller is connected with USB it cannot be polled
	    printf("Controller %d is connected with USB, cannot poll data", j);
	}
	
    controllers[j] = psmove_connect_by_id(j);	//Connect to the controller
	assert(controllers[j] != NULL);				//Check that the controller actually connected
	psmove_set_rate_limiting(controllers[j], PSMove_True);//Rate limit the controller, should be on by default but will enable just in case
	psmove_set_leds(controllers[j], r[j], g[j], b[j]);//Assign colot to the controller LED
	psmove_update_leds(controllers[j]);			//Update/turn on the controller LED
	
	}
	
	if (controllers == NULL) {	//In case no controller successfully connected
        printf("Could not connect to default Move controller.\n"
               "Please connect one via Bluetooth.\n");
        exit(1);
    }
	
   while ((cvWaitKey(1) & 0xFF) != 27) {	//Press q to exit program

   
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

		Button_Data = psmove_get_buttons(controllers[j]);	//Get button data
		printf("Controller %d: buttons: %x\n", j, Button_Data);	//Print button data
        
		}
		std::string Cntrl_ID = psmove_get_serial(controllers[j]);
		
		const char * Chr_cntrl_id = Cntrl_ID.c_str();
		
		StringBuffer s;
		Writer<StringBuffer> writer(s);
		
		writer.StartObject();
		writer.Key("ID");
		writer.String(Chr_cntrl_id);
		writer.Key("Acc_x");
		writer.Uint(Acc_Data[0]);
		writer.Key("Acc_y");
		writer.Uint(Acc_Data[1]);
		writer.Key("Acc_z");
		writer.Uint(Acc_Data[2]);
		writer.Key("Gyro_x");
		writer.Uint(Gyro_Data[0]);
		writer.Key("Gyro_y");
		writer.Uint(Gyro_Data[1]);
		writer.Key("Gyro_z");
		writer.Uint(Gyro_Data[2]);
		writer.Key("Mag_x");
		writer.Uint(Mag_Data[0]);
		writer.Key("Mag_y");
		writer.Uint(Mag_Data[1]);
		writer.Key("Mag_z");
		writer.Uint(Mag_Data[2]);
		writer.Key("Button");
		writer.Uint(Button_Data);
		writer.EndObject();
		
		
		/*
		std::string msg = "Controller ";
		std::string nr = std::to_string(j);
		std::string Acc_x = std::to_string(Acc_Data[0]);
		std::string Acc_y = std::to_string(Acc_Data[1]);
		std::string Acc_z = std::to_string(Acc_Data[2]);
		std::string Gyro_x = std::to_string(Gyro_Data[0]);
		std::string Gyro_y = std::to_string(Gyro_Data[1]);
		std::string Gyro_z = std::to_string(Gyro_Data[2]);
		std::string Mag_x = std::to_string(Mag_Data[0]);
		std::string Mag_y = std::to_string(Mag_Data[1]);
		std::string Mag_z = std::to_string(Mag_Data[2]);
		std::string button_str = std::to_string(Button_Data);
		
		message = msg + nr + ": " + Acc_x + " " + Acc_y + Acc_z + " " + 
		Gyro_x + ": " + Gyro_y + ": " + Gyro_z + ": " + 
		Mag_x + ": " + Mag_y + ": " + Mag_z + ": " + button_str;
		*/
		
		
		endpoint.send(id, s.GetString());
		
	}
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
	
   return 0;
}

