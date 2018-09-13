// Example program
#include <iostream>
#include <string>
#include <variant>
#include <fstream>
#include <algorithm>
#include <unordered_map>


bool is_number(const std::string &c) 
{

  return (c.find_first_of("0123456789") != std::string::npos);
}

bool is_boolean(std::string &c) 
{

  std::transform(c.begin(), c.end(), c.begin(), ::tolower);
  std::cout << "c: " << c << std::endl;
  return (c == "false" || c == "true");

}

bool convert_to_boolean(const std::string &c) 
{

  bool conversion = false;

  if (c == "true")
    conversion = true;

  return conversion;

}

bool erase_white_space(std::string &c) 
{
  c.erase( remove( c.begin(), c.end(), ' ' ), c.end() );
}


int main()
{
    
    
   std::string filename = "/home/mark/Downloads/test.txt";
   std::ifstream inFile(filename);

   std::string strOneLine;

   std::unordered_map<std::string,std::variant<std::string,bool,float>> params;

   std::string manager_name;
   std::string plugin_name;
   std::string var_name;
   std::string var_value_s;
   float var_value_f;
   bool var_value_b;

   int iter_length1, iter_length2, iter_length3;
   
   inFile >> std::ws;
   while (!inFile.eof()) {

      std::getline(inFile,manager_name,'/');
      inFile >> std::ws;

      if (!manager_name.empty()) {
        std::getline(inFile,plugin_name,'/');
        std::getline(inFile,var_name,':');
        std::getline(inFile,var_value_s,'\n');
        inFile >> std::ws;
        // var_value_f = std::strtof(var_value_s.c_str(), NULL); 

        // remove white space from the strings
        erase_white_space(manager_name);
        erase_white_space(var_name);
        erase_white_space(var_value_s);


        std::cout << "manager_name: " << manager_name << std::endl;
        std::cout << "plugin_name: " << plugin_name << std::endl;
        std::cout << "var_name: " << var_name << std::endl;
        // std::cout <<  "value_f: " << var_value_f << std::endl;
        std::cout << "is number: " << is_number(var_value_s) << std::endl;
        std::cout << "is boolean: " << is_boolean(var_value_s) << std::endl;
      }


   }



    inFile.close();
    
    

}