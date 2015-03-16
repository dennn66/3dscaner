#include <inifile++.hpp>
#include "capturelib.hpp"

/**
 *  Main function.
 */
int main(int argc, char *argv[])
{

  int result = -1;

  if (argc == 2) {
	try {
		inifilepp::parser p("scaner.ini");
		const inifilepp::parser::entry *ent;
		char tty[255]="/dev/ttyUSB0";
		int baud=115200;
		while((ent = p.next())) {
			if(strcmp(ent->sect, "Port")==0){
				if(strcmp(ent->key, "tty")==0) strcpy(tty, ent->key);
				if(strcmp(ent->key, "baud")==0) baud = atoi(ent->val);
			}
		}
 		result = Scan(tty, baud, "./images/new/");
	} catch(inifilepp::parser::exception &e) {
		std::cerr << "parse error at (" << e.line << "," << e.cpos << ")\n";

	}
  } 
  if (argc == 3) {
  	result = Scan(argv[1], atoi(argv[2]), "./images/new/");
  } 
  if (argc == 4) {
  	result = Scan(argv[1], atoi(argv[2]), argv[1]);
  }   
  if(result !=0) {
	std::cerr << "Canceled" << std::endl;
	return EXIT_SUCCESS;
  }	

  std::cerr << "Done" << std::endl;
  return EXIT_SUCCESS;
}

