#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

using namespace curlpp::options;

int main(int argc, char *argv[])
{
	std::ostringstream os;
	std::string url;
	double freq = 1;
	while(1){  
   		try{
	       		if (argc == 2){
        			url = argv[1];
  			}
        		else{
           			url = "155.207.33.189:1026";
        		}
        // That's all that is needed to do cleanup of used resources (RAII sty$	      
			os.clear();
		        curlpp::Cleanup myCleanup;
    		 	// Our request to be sent.
        		curlpp::Easy myRequest;
    		  	// Set the URL.
        		// entities/{entityId}?type=&attrs=
        		myRequest.setOpt<Url>(url + "/v2/entities/Point/attrs?type=Point");
    
			// Send request and get a result.
			// By default the result goes to standard output.
			curlpp::options::WriteStream ws(&os);
		        myRequest.setOpt(ws);
		        myRequest.perform();
		        std::string str1 = os.str();	
			std::string str2 = str1.substr(31,1); 
		        if (str2 != ","){
		            std::cout << "ALERT!! Detected person at: ";
			    std::cout << str1 << std::endl;
		        }

		}
		catch(curlpp::RuntimeError & e){
		       	std::cout << e.what() << std::endl;
      		}
		catch(curlpp::LogicError & e){
			std::cout << e.what() << std::endl;
      		}
      	sleep(1/freq);
	}
	return 0;
}
